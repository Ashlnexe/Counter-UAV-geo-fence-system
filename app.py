# =============================================================================
# app.py — Counter-UAV Geo-Fence Monitor  (UI v2)
# Aesthetic: Tactical Operations Centre — industrial precision, dense info
# Fonts: Share Tech Mono (telemetry data) · Rajdhani (labels/headings)
# =============================================================================

import os
import math

from dash import Dash, html, dcc, Input, Output, clientside_callback
import plotly.graph_objects as go

from config import BASE_LAT, BASE_LON, ZONES
from geofence import geofence_manager
from adapters import SimulatorAdapter, CSVReplayAdapter, MAVLinkAdapter, OpenSkyAdapter
from core_engine import tracking_engine

# ── Data source bootstrap ─────────────────────────────────────────────────────
DATA_SOURCE = os.getenv("UAV_DATA_SOURCE", "simulation").lower()

_ADAPTERS = {
    "simulation": lambda: SimulatorAdapter(),
    "csv":        lambda: CSVReplayAdapter(os.getenv("UAV_LOG_PATH", "logs/sample_flight.csv")),
    "mavlink":    lambda: MAVLinkAdapter(os.getenv("UAV_LOG_PATH", "logs/flight.tlog")),
    "opensky":    lambda: OpenSkyAdapter(poll_interval=10),
}
if DATA_SOURCE not in _ADAPTERS:
    raise ValueError(f"Unknown UAV_DATA_SOURCE: {DATA_SOURCE!r}")

tracking_engine.start([_ADAPTERS[DATA_SOURCE]()])
get_state = tracking_engine.get_state

# ── Colour tokens ─────────────────────────────────────────────────────────────
C = {
    "bg":     "#050810",
    "panel":  "rgba(7, 14, 28, 0.92)",
    "border": "rgba(0, 210, 255, 0.10)",
    "cyan":   "#00d2ff",
    "green":  "#00f090",
    "red":    "#ff2d55",
    "orange": "#ff8c00",
    "yellow": "#ffd60a",
    "purple": "#bf5af2",
    "text1":  "#ccd8ef",
    "text2":  "#3d5a82",
    "text3":  "#7a96b8",
}

DRONE_COLORS = {
    "UAV-01": "#00d2ff",
    "UAV-02": "#ff8c00",
    "UAV-03": "#ff2d55",
    "UAV-04": "#bf5af2",
}

SEV = {"CRITICAL": C["red"], "MEDIUM": C["orange"], "LOW": C["yellow"]}

SOURCE_BADGE = {
    "simulation": ("SIMULATION",  C["purple"]),
    "csv":        ("CSV REPLAY",  C["cyan"]),
    "mavlink":    ("MAVLink LOG", C["orange"]),
    "opensky":    ("LIVE ADS-B",  C["green"]),
}

FONTS = (
    "https://fonts.googleapis.com/css2?"
    "family=Share+Tech+Mono&"
    "family=Rajdhani:wght@400;600;700&"
    "display=swap"
)

app = Dash(__name__, title="Counter-UAV GFS", external_stylesheets=[FONTS])

# ─────────────────────────────────────────────────────────────────────────────
# Style helpers
# ─────────────────────────────────────────────────────────────────────────────
MONO = {"fontFamily": "'Share Tech Mono', monospace"}
SANS = {"fontFamily": "'Rajdhani', sans-serif"}


def panel(extra=None):
    s = {
        "backgroundColor": C["panel"],
        "border": f"1px solid {C['border']}",
        "borderRadius": "5px",
        "backdropFilter": "blur(6px)",
    }
    if extra:
        s.update(extra)
    return s


def label(txt, color=None, size="9px", **kw):
    return html.Div(txt, style={**SANS, "color": color or C["text2"],
                                 "fontSize": size, "fontWeight": "700",
                                 "letterSpacing": "0.10em",
                                 "textTransform": "uppercase", **kw})


# ─────────────────────────────────────────────────────────────────────────────
# KPI card
# ─────────────────────────────────────────────────────────────────────────────
def kpi(eid, lbl, dflt, accent):
    return html.Div(
        style={**panel(), "flex": "1", "padding": "10px 16px",
               "borderTop": f"2px solid {accent}"},
        children=[
            label(lbl, accent),
            html.Div(id=eid, children=dflt,
                     style={**MONO, "fontSize": "28px", "color": accent,
                            "marginTop": "4px",
                            "textShadow": f"0 0 14px {accent}66"}),
        ]
    )


# ─────────────────────────────────────────────────────────────────────────────
# Sidebar zone breach row
# ─────────────────────────────────────────────────────────────────────────────
def zone_row(zone_id, color, eid):
    return html.Div(
        style={"display": "flex", "justifyContent": "space-between",
               "alignItems": "center", "padding": "6px 8px",
               "marginBottom": "3px", "borderRadius": "3px",
               "backgroundColor": f"{color}0c",
               "border": f"1px solid {color}1f"},
        children=[
            html.Div(style={"display": "flex", "alignItems": "center", "gap": "7px"},
                     children=[
                         html.Div(style={"width": "5px", "height": "5px",
                                          "borderRadius": "50%",
                                          "backgroundColor": color,
                                          "boxShadow": f"0 0 5px {color}"}),
                         html.Span(zone_id, style={**SANS, "fontSize": "9px",
                                                    "fontWeight": "700", "color": color,
                                                    "letterSpacing": "0.08em"}),
                     ]),
            html.Span(id=eid, children="0",
                      style={**MONO, "fontSize": "12px", "color": color}),
        ]
    )


# ─────────────────────────────────────────────────────────────────────────────
# Stat cell used inside drone cards
# ─────────────────────────────────────────────────────────────────────────────
def _stat(lbl, val):
    return html.Div([
        html.Div(lbl, style={**SANS, "fontSize": "8px", "color": C["text2"],
                              "fontWeight": "700", "letterSpacing": "0.08em"}),
        html.Div(val, style={**MONO, "fontSize": "12px", "color": C["text1"],
                              "marginTop": "1px"}),
    ])


# ─────────────────────────────────────────────────────────────────────────────
# Map geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def _circle(lat, lon, radius_m, n=40):
    """Generate lat/lon polygon for a 3-sigma uncertainty ring."""
    lats, lons = [], []
    cos_lat = math.cos(math.radians(lat)) + 1e-10
    for i in range(n + 1):
        a = math.radians(i * 360 / n)
        lats.append(lat + (radius_m * math.cos(a)) / 110570.0)
        lons.append(lon + (radius_m * math.sin(a)) / (111320.0 * cos_lat))
    return lats, lons


def _vector_end(lat, lon, heading_deg, speed_mps, t=12.0):
    """Project position forward by t seconds at current heading and speed."""
    d = speed_mps * t
    hr = math.radians(heading_deg)
    cos_lat = math.cos(math.radians(lat)) + 1e-10
    return (
        lat + (d * math.cos(hr)) / 110570.0,
        lon + (d * math.sin(hr)) / (111320.0 * cos_lat),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Sidebar
# ─────────────────────────────────────────────────────────────────────────────
src_lbl, src_col = SOURCE_BADGE.get(DATA_SOURCE, ("UNKNOWN", C["text2"]))

sidebar = html.Div(
    style={**panel(), "width": "190px", "flexShrink": "0",
           "padding": "14px 10px", "display": "flex",
           "flexDirection": "column", "gap": "14px"},
    children=[
        html.Div([
            label("System"),
            html.Div(style={"display": "flex", "alignItems": "center",
                             "gap": "6px", "marginTop": "7px"},
                     children=[
                         html.Div(style={"width": "7px", "height": "7px",
                                          "borderRadius": "50%",
                                          "backgroundColor": C["green"],
                                          "boxShadow": f"0 0 8px {C['green']}"}),
                         html.Span("OPERATIONAL",
                                   style={**SANS, "fontSize": "10px", "fontWeight": "700",
                                          "color": C["green"], "letterSpacing": "0.08em"}),
                     ]),
        ]),

        html.Hr(style={"border": "none", "borderTop": f"1px solid {C['border']}", "margin": "0"}),

        html.Div([
            label("Data Source"),
            html.Div(src_lbl,
                     style={**MONO, "fontSize": "11px", "color": src_col,
                            "marginTop": "7px", "padding": "3px 8px",
                            "border": f"1px solid {src_col}44",
                            "borderRadius": "3px", "backgroundColor": f"{src_col}0f",
                            "display": "inline-block"}),
        ]),

        html.Hr(style={"border": "none", "borderTop": f"1px solid {C['border']}", "margin": "0"}),

        html.Div([
            label("Zone Breaches"),
            html.Div(style={"marginTop": "8px"},
                     children=[
                         zone_row("EXCLUSION", C["red"],    "zc-excl"),
                         zone_row("BUFFER",    C["orange"], "zc-buf"),
                         zone_row("MONITORED", C["yellow"], "zc-mon"),
                     ]),
        ]),

        html.Hr(style={"border": "none", "borderTop": f"1px solid {C['border']}", "margin": "0"}),

        html.Div([
            label("Active Tracks"),
            html.Div(id="sidebar-tracks",
                     style={"marginTop": "8px", "display": "flex",
                            "flexDirection": "column", "gap": "3px"}),
        ]),

        html.Div(style={"flex": "1"}),

        html.Div([
            label("Fence Centre"),
            html.Div(f"{BASE_LAT}°N",
                     style={**MONO, "fontSize": "10px", "color": C["text3"], "marginTop": "5px"}),
            html.Div(f"{BASE_LON}°E",
                     style={**MONO, "fontSize": "10px", "color": C["text3"]}),
        ]),
    ]
)


# ─────────────────────────────────────────────────────────────────────────────
# Alert panel
# ─────────────────────────────────────────────────────────────────────────────
alert_panel = html.Div(
    style={**panel(), "width": "275px", "flexShrink": "0",
           "padding": "12px", "display": "flex", "flexDirection": "column"},
    children=[
        html.Div(style={"display": "flex", "justifyContent": "space-between",
                         "alignItems": "center", "marginBottom": "10px"},
                 children=[
                     html.Span("THREAT LOG",
                               style={**SANS, "fontSize": "10px", "fontWeight": "700",
                                      "color": C["text1"], "letterSpacing": "0.12em"}),
                     html.Span(id="feed-counter", children="0 events",
                               style={**MONO, "fontSize": "10px", "color": C["cyan"],
                                      "backgroundColor": f"{C['cyan']}13",
                                      "padding": "2px 6px", "borderRadius": "10px",
                                      "border": f"1px solid {C['cyan']}30"}),
                 ]),
        html.Div(id="alert-feed-container",
                 style={"flex": "1", "overflowY": "auto",
                        "display": "flex", "flexDirection": "column", "gap": "5px"},
                 children=[
                     html.Div("Awaiting target detections…",
                              style={"color": C["text2"], "textAlign": "center",
                                     "marginTop": "30px", "fontSize": "11px",
                                     "fontStyle": "italic"})
                 ]),
    ]
)


# ─────────────────────────────────────────────────────────────────────────────
# Layout
# ─────────────────────────────────────────────────────────────────────────────
app.layout = html.Div(
    style={"backgroundColor": C["bg"], "color": C["text1"],
           "minHeight": "100vh", "padding": "12px",
           "boxSizing": "border-box", **SANS,
           "display": "flex", "flexDirection": "column", "gap": "10px"},
    children=[

        # ── Header ────────────────────────────────────────────────────────
        html.Div(
            style={"display": "flex", "alignItems": "center",
                   "justifyContent": "space-between",
                   "paddingBottom": "10px",
                   "borderBottom": f"1px solid {C['border']}"},
            children=[
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "12px"},
                         children=[
                             html.Div(
                                 style={"width": "34px", "height": "34px",
                                        "borderRadius": "50%",
                                        "border": f"2px solid {C['cyan']}",
                                        "display": "flex", "alignItems": "center",
                                        "justifyContent": "center", "fontSize": "17px",
                                        "boxShadow": f"0 0 14px {C['cyan']}44",
                                        "flexShrink": "0"},
                                 children="🛡"
                             ),
                             html.Div([
                                 html.Div("COUNTER-UAV GEO-FENCE MONITOR",
                                          style={"fontSize": "17px", "fontWeight": "700",
                                                 "letterSpacing": "0.07em", "color": C["cyan"],
                                                 "textShadow": f"0 0 18px {C['cyan']}44"}),
                                 html.Div(
                                     "6D OOSM Kalman Filter  ·  3-Zone IDS  ·  "
                                     "Multi-Adapter Telemetry Pipeline  ·  C++ Eigen3 Core",
                                     style={"fontSize": "9px", "color": C["text2"],
                                            "letterSpacing": "0.05em", "marginTop": "2px"}
                                 ),
                             ]),
                         ]),
                html.Div(style={"display": "flex", "alignItems": "center", "gap": "10px"},
                         children=[
                             html.Div(id="live-clock",
                                      style={**MONO, "fontSize": "16px",
                                             "color": C["text3"], "letterSpacing": "0.04em"}),
                             html.Div(
                                 style={"display": "flex", "alignItems": "center", "gap": "6px",
                                        "backgroundColor": f"{C['green']}10",
                                        "border": f"1px solid {C['green']}40",
                                        "padding": "5px 12px", "borderRadius": "20px"},
                                 children=[
                                     html.Div(style={"width": "6px", "height": "6px",
                                                      "borderRadius": "50%",
                                                      "backgroundColor": C["green"],
                                                      "boxShadow": f"0 0 7px {C['green']}"}),
                                     html.Span("SYSTEM LIVE",
                                               style={**SANS, "fontSize": "9px",
                                                      "fontWeight": "700", "color": C["green"],
                                                      "letterSpacing": "0.12em"}),
                                 ]
                             ),
                         ]),
            ]
        ),

        # ── KPI Strip ─────────────────────────────────────────────────────
        html.Div(
            style={"display": "flex", "gap": "8px"},
            children=[
                kpi("kpi-active-drones",   "Tracked Targets",   "0", C["cyan"]),
                kpi("kpi-confirmed",        "Confirmed Tracks",  "0", C["green"]),
                kpi("kpi-total-alerts",    "Alert Events",      "0", C["orange"]),
                kpi("kpi-critical-alerts", "Critical Breaches", "0", C["red"]),
            ]
        ),

        # ── Main body: sidebar | map | alert panel ─────────────────────────
        html.Div(
            style={"display": "flex", "gap": "10px"},
            children=[
                sidebar,
                html.Div(
                    style={**panel(), "flex": "1", "padding": "10px",
                           "display": "flex", "flexDirection": "column"},
                    children=[
                        html.Div(
                            style={"display": "flex", "justifyContent": "space-between",
                                   "alignItems": "center", "marginBottom": "8px"},
                            children=[
                                html.Span("TACTICAL RADAR VIEW",
                                          style={**SANS, "fontSize": "10px", "fontWeight": "700",
                                                 "color": C["text1"], "letterSpacing": "0.12em"}),
                                html.Div([
                                    html.Span("── ", style={"color": "#ffffff22", "fontSize": "9px"}),
                                    html.Span("Raw GPS   ", style={"color": C["text2"], "fontSize": "9px"}),
                                    html.Span("── ", style={"color": C["cyan"], "fontSize": "9px"}),
                                    html.Span("Kalman   ", style={"color": C["text3"], "fontSize": "9px"}),
                                    html.Span("◌  ", style={"color": "#ffffff33", "fontSize": "11px"}),
                                    html.Span("3σ ring   ", style={"color": C["text2"], "fontSize": "9px"}),
                                    html.Span("→  ", style={"color": C["text2"], "fontSize": "9px"}),
                                    html.Span("12s vector", style={"color": C["text2"], "fontSize": "9px"}),
                                ]),
                            ]
                        ),
                        dcc.Graph(id="radar-map",
                                  style={"height": "60vh"},
                                  config={"displayModeBar": False}),
                    ]
                ),
                alert_panel,
            ]
        ),

        # ── Drone cards ───────────────────────────────────────────────────
        html.Div(id="drone-cards",
                 style={"display": "flex", "gap": "8px", "flexWrap": "wrap"}),

        dcc.Interval(id="live-interval", interval=1000, n_intervals=0),
    ]
)


# ─────────────────────────────────────────────────────────────────────────────
# Clientside clock — zero server round-trips
# ─────────────────────────────────────────────────────────────────────────────
clientside_callback(
    """
    function(n) {
        const now = new Date();
        const pad = x => String(x).padStart(2, '0');
        return pad(now.getUTCHours()) + ':' + pad(now.getUTCMinutes()) + ':'
               + pad(now.getUTCSeconds()) + ' UTC';
    }
    """,
    Output("live-clock", "children"),
    Input("live-interval", "n_intervals"),
)


# ─────────────────────────────────────────────────────────────────────────────
# Map builder
# ─────────────────────────────────────────────────────────────────────────────
def _build_map(drones):
    fig = go.Figure()

    for d in drones:
        color = DRONE_COLORS.get(d["id"], "#ffffff")

        # Faint raw-GPS track
        if len(d.get("true_path", [])) > 1:
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lon=[p[1] for p in d["true_path"]],
                lat=[p[0] for p in d["true_path"]],
                line=dict(width=1, color="#ffffff"),
                opacity=0.15, hoverinfo="none", showlegend=False,
            ))

        # Kalman-filtered track
        if len(d.get("path", [])) > 1:
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lon=[p[1] for p in d["path"]],
                lat=[p[0] for p in d["path"]],
                line=dict(width=2, color=color),
                opacity=0.85, hoverinfo="none", showlegend=False,
            ))

        # 3-sigma uncertainty ring
        r = d.get("uncertainty_radius_m", 0)
        if r > 5:
            clats, clons = _circle(d["lat"], d["lon"], r)
            fig.add_trace(go.Scattermapbox(
                mode="lines", lat=clats, lon=clons,
                line=dict(width=1, color=color),
                opacity=0.28, hoverinfo="none", showlegend=False,
            ))

        # 12-second heading vector
        if d["speed"] > 0.5:
            elat, elon = _vector_end(d["lat"], d["lon"], d["heading"], d["speed"])
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lat=[d["lat"], elat], lon=[d["lon"], elon],
                line=dict(width=2, color=color),
                opacity=0.50, hoverinfo="none", showlegend=False,
            ))

    # Drone position markers
    if drones:
        fig.add_trace(go.Scattermapbox(
            mode="markers+text",
            lat=[d["lat"] for d in drones],
            lon=[d["lon"] for d in drones],
            marker=dict(size=13,
                        color=[DRONE_COLORS.get(d["id"], "#fff") for d in drones]),
            text=[d["id"] for d in drones],
            textposition="top right",
            textfont=dict(color="#ffffff", size=10, family="Share Tech Mono"),
            hoverinfo="text",
            hovertext=[
                f"<b>{d['id']}</b><br>"
                f"Alt: {d['alt']} m  |  Spd: {d['speed']} m/s<br>"
                f"Hdg: {d['heading']}°  |  3σ: {round(d.get('uncertainty_radius_m', 0), 1)} m<br>"
                f"{round(d['lat'], 5)}°N  {round(d['lon'], 5)}°E"
                for d in drones
            ],
            showlegend=False,
        ))

    fig.update_layout(
        mapbox=dict(
            style="carto-darkmatter",
            center=dict(lat=BASE_LAT, lon=BASE_LON),
            zoom=12.2,
            layers=geofence_manager.get_mapbox_layers(),
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        uirevision="constant",
    )
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# Alert feed builder
# ─────────────────────────────────────────────────────────────────────────────
def _build_feed(alerts):
    if not alerts:
        return [html.Div("No alerts. Airspace nominal.",
                         style={"color": C["green"], "textAlign": "center",
                                "marginTop": "30px", "fontSize": "11px",
                                **SANS})]
    cards = []
    for a in alerts[:60]:
        col = SEV.get(a["severity"], C["yellow"])
        drone_col = DRONE_COLORS.get(a["drone_id"], C["text1"])
        cards.append(html.Div(
            style={"borderLeft": f"3px solid {col}",
                   "backgroundColor": f"{col}0a",
                   "padding": "7px 9px", "borderRadius": "0 3px 3px 0"},
            children=[
                html.Div(style={"display": "flex", "justifyContent": "space-between",
                                 "marginBottom": "3px"},
                         children=[
                             html.Span(a["threat_type"].replace("_", " "),
                                       style={**SANS, "fontSize": "10px", "fontWeight": "700",
                                              "color": col, "letterSpacing": "0.06em"}),
                             html.Span(a["timestamp"],
                                       style={**MONO, "fontSize": "9px", "color": C["text2"]}),
                         ]),
                html.Div([
                    html.Span(a["drone_id"],
                              style={**MONO, "fontSize": "11px",
                                     "color": drone_col, "marginRight": "5px"}),
                    html.Span(f"→ {a['zone_breached']}",
                              style={**MONO, "fontSize": "11px", "color": C["text3"]}),
                ]),
                html.Div(
                    f"{a['speed']} m/s  ·  {a.get('alt', '—')} m  ·  {a['lat']}, {a['lon']}",
                    style={**MONO, "fontSize": "9px", "color": C["text2"], "marginTop": "2px"}
                ),
            ]
        ))
    return cards


# ─────────────────────────────────────────────────────────────────────────────
# Drone cards (replaces flat telemetry table)
# ─────────────────────────────────────────────────────────────────────────────
def _build_drone_cards(drones, alerts):
    if not drones:
        return [html.Div("No confirmed tracks.",
                         style={"color": C["text2"], "fontSize": "11px", **SANS, "padding": "8px"})]

    cards = []
    for d in drones:
        col = DRONE_COLORS.get(d["id"], C["text1"])
        d_alerts = [a for a in alerts if a["drone_id"] == d["id"]]
        if d_alerts:
            sev = d_alerts[0]["severity"]
            status_txt = {"CRITICAL": "CRITICAL", "MEDIUM": "WARNING"}.get(sev, "ALERT")
            status_col = SEV.get(sev, C["yellow"])
        else:
            status_txt, status_col = "NOMINAL", C["green"]

        u_r = d.get("uncertainty_radius_m", 0)

        cards.append(html.Div(
            style={**panel(), "minWidth": "185px", "flex": "1",
                   "padding": "12px", "borderTop": f"2px solid {col}"},
            children=[
                html.Div(
                    style={"display": "flex", "justifyContent": "space-between",
                           "alignItems": "flex-start", "marginBottom": "10px"},
                    children=[
                        html.Div([
                            html.Div(d["id"],
                                     style={**MONO, "fontSize": "15px", "color": col,
                                            "textShadow": f"0 0 10px {col}55"}),
                            html.Div(
                                d.get("source", "SIM").upper(),
                                style={**SANS, "fontSize": "9px", "color": C["text2"],
                                       "letterSpacing": "0.06em", "marginTop": "2px"}
                            ),
                        ]),
                        html.Span(
                            status_txt,
                            style={**SANS, "fontSize": "9px", "fontWeight": "700",
                                   "color": status_col,
                                   "backgroundColor": f"{status_col}18",
                                   "border": f"1px solid {status_col}44",
                                   "padding": "2px 7px", "borderRadius": "3px",
                                   "letterSpacing": "0.06em"}
                        ),
                    ]
                ),
                html.Div(
                    style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "6px"},
                    children=[
                        _stat("SPEED",   f"{d['speed']} m/s"),
                        _stat("ALT",     f"{d['alt']} m"),
                        _stat("HDG",     f"{d['heading']}°"),
                        _stat("3σ RAD",  f"{round(u_r, 0):.0f} m"),
                    ]
                ),
            ]
        ))
    return cards


# ─────────────────────────────────────────────────────────────────────────────
# Sidebar track list
# ─────────────────────────────────────────────────────────────────────────────
def _build_sidebar_tracks(drones):
    if not drones:
        return [html.Div("—", style={"color": C["text2"], "fontSize": "11px"})]
    items = []
    for d in drones:
        col = DRONE_COLORS.get(d["id"], C["text1"])
        items.append(html.Div(
            style={"display": "flex", "alignItems": "center", "gap": "7px",
                   "padding": "4px 6px", "borderRadius": "3px",
                   "backgroundColor": f"{col}0c",
                   "border": f"1px solid {col}20"},
            children=[
                html.Div(style={"width": "5px", "height": "5px", "borderRadius": "50%",
                                 "backgroundColor": col, "boxShadow": f"0 0 5px {col}"}),
                html.Span(d["id"], style={**MONO, "fontSize": "11px", "color": col}),
                html.Span(f"{d['alt']} m",
                          style={**MONO, "fontSize": "9px",
                                 "color": C["text2"], "marginLeft": "auto"}),
            ]
        ))
    return items


# ─────────────────────────────────────────────────────────────────────────────
# Main callback
# ─────────────────────────────────────────────────────────────────────────────
@app.callback(
    Output("radar-map",            "figure"),
    Output("kpi-active-drones",    "children"),
    Output("kpi-confirmed",         "children"),
    Output("kpi-total-alerts",     "children"),
    Output("kpi-critical-alerts",  "children"),
    Output("feed-counter",         "children"),
    Output("alert-feed-container", "children"),
    Output("drone-cards",          "children"),
    Output("sidebar-tracks",       "children"),
    Output("zc-excl",              "children"),
    Output("zc-buf",               "children"),
    Output("zc-mon",               "children"),
    Input("live-interval",         "n_intervals"),
)
def update(n):
    state  = get_state()
    drones = state["drones"]
    alerts = state["alerts"]

    total_a = len(alerts)
    crit_a  = sum(1 for a in alerts if a["severity"] == "CRITICAL")

    zc = {"EXCLUSION": 0, "BUFFER": 0, "MONITORED": 0}
    for a in alerts:
        if a["zone_breached"] in zc:
            zc[a["zone_breached"]] += 1

    return (
        _build_map(drones),
        str(len(drones)),
        str(len(drones)),          # all returned by get_state are confirmed
        str(total_a),
        str(crit_a),
        f"{total_a} events",
        _build_feed(alerts),
        _build_drone_cards(drones, alerts),
        _build_sidebar_tracks(drones),
        str(zc["EXCLUSION"]),
        str(zc["BUFFER"]),
        str(zc["MONITORED"]),
    )


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app.run(debug=False, port=8050, host="0.0.0.0")
