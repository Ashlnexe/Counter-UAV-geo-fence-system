# =============================================================================
# app.py — Dash dashboard for Counter-UAV Geo-Fence Monitoring System
# =============================================================================

from dash import Dash, html, dcc, Input, Output
import plotly.graph_objects as go

from config import BASE_LAT, BASE_LON, ZONES
from geofence import geofence_manager
import os

DATA_SOURCE = os.getenv("UAV_DATA_SOURCE", "simulation")  # "simulation" | "csv" | "mavlink"

if DATA_SOURCE == "simulation":
    from simulation import simulation_engine
    get_state = simulation_engine.get_state
else:
    from data_ingestion import open_log
    from ingestion_adapter import IngestionAdapter
    adapter = IngestionAdapter(open_log(os.getenv("UAV_LOG_PATH"), drone_id="UAV-REAL"))
    get_state = adapter.get_state

app = Dash(__name__, title="Counter-UAV Geo-Fence Monitor")

# Per-drone accent colours
DRONE_COLORS = {
    "UAV-01": "#00f2ff",   # Cyan
    "UAV-02": "#ff9f1a",   # Orange
    "UAV-03": "#ff2a2a",   # Red
    "UAV-04": "#bc13fe",   # Purple
}

# =============================================================================
# Helpers (must be defined before layout)
# =============================================================================

def _panel_style() -> dict:
    return {
        "backgroundColor": "rgba(20,27,45,0.6)",
        "border": "1px solid rgba(255,255,255,0.08)",
        "borderRadius": "10px",
        "padding": "15px",
        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)",
    }


def _kpi_card(element_id: str, label: str, default: str, color: str) -> html.Div:
    return html.Div(
        style={"flex": "1", **_panel_style()},
        children=[
            html.Div(label, style={"fontSize": "12px", "color": "#94a3b8", "fontWeight": "600"}),
            html.Div(id=element_id, children=default,
                     style={"fontSize": "28px", "fontWeight": "bold", "color": color, "marginTop": "5px"}),
        ]
    )


# =============================================================================
# Layout
# =============================================================================

app.layout = html.Div(
    style={
        "backgroundColor": "#0b0f19",
        "color": "#f1f5f9",
        "fontFamily": "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
        "minHeight": "100vh",
        "padding": "20px",
        "boxSizing": "border-box",
        "overflowX": "hidden",
    },
    children=[

        # ── Header ────────────────────────────────────────────────────────────
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "borderBottom": "1px solid rgba(255,255,255,0.1)",
                "paddingBottom": "15px",
                "marginBottom": "20px",
            },
            children=[
                html.Div([
                    html.H1(
                        "🛡️ COUNTER-UAV GEO-FENCE MONITOR",
                        style={
                            "margin": 0,
                            "fontSize": "26px",
                            "fontWeight": "800",
                            "letterSpacing": "1px",
                            "color": "#00f2ff",
                            "textShadow": "0 0 10px rgba(0,242,255,0.3)",
                        }
                    ),
                    html.Div(
                        "Kalman-filtered multi-zone aerial threat tracking  |  Bengaluru 12.97°N 77.59°E",
                        style={"fontSize": "12px", "color": "#94a3b8", "marginTop": "4px"}
                    ),
                ]),
                html.Div(
                    style={
                        "display": "flex", "alignItems": "center",
                        "backgroundColor": "rgba(16,185,129,0.1)",
                        "border": "1px solid #10b981",
                        "padding": "6px 14px", "borderRadius": "20px",
                    },
                    children=[
                        html.Div(style={
                            "width": "8px", "height": "8px",
                            "backgroundColor": "#10b981", "borderRadius": "50%",
                            "marginRight": "8px", "boxShadow": "0 0 8px #10b981",
                        }),
                        html.Span("SYSTEM LIVE", style={
                            "fontSize": "12px", "fontWeight": "bold",
                            "color": "#10b981", "letterSpacing": "1px",
                        }),
                    ]
                ),
            ]
        ),

        # ── KPI Cards ─────────────────────────────────────────────────────────
        html.Div(
            style={"display": "flex", "gap": "20px", "marginBottom": "20px"},
            children=[
                _kpi_card("kpi-active-drones",  "TRACKED AERIAL TARGETS",   "4",  "#00f2ff"),
                _kpi_card("kpi-total-alerts",   "TOTAL ALERTS GENERATED",   "0",  "#ff9f1a"),
                _kpi_card("kpi-critical-alerts","CRITICAL EXCLUSION BREACHES","0", "#ff2a2a"),
            ]
        ),

        # ── Map + Feed ────────────────────────────────────────────────────────
        html.Div(
            style={"display": "flex", "gap": "20px", "alignItems": "stretch"},
            children=[

                # Left: tactical map
                html.Div(
                    style={
                        "flex": "7",
                        **_panel_style(),
                        "display": "flex", "flexDirection": "column",
                    },
                    children=[
                        html.Div(
                            style={"display": "flex", "justifyContent": "space-between",
                                   "marginBottom": "10px", "alignItems": "center"},
                            children=[
                                html.Div("🛰️ TACTICAL RADAR VIEW",
                                         style={"fontSize": "14px", "fontWeight": "bold", "color": "#e2e8f0"}),
                                html.Div(
                                    [
                                        html.Span("── ", style={"color": "#ffffff", "opacity": "0.4"}),
                                        html.Span("Raw GPS   ", style={"color": "#ffffff", "opacity": "0.4", "fontSize": "11px"}),
                                        html.Span("── ", style={"color": "#00f2ff"}),
                                        html.Span("Kalman filtered", style={"color": "#00f2ff", "fontSize": "11px"}),
                                    ]
                                ),
                            ]
                        ),
                        dcc.Graph(
                            id="radar-map",
                            style={"height": "65vh", "width": "100%"},
                            config={"displayModeBar": False},
                        ),
                    ]
                ),

                # Right: threat log
                html.Div(
                    style={
                        "flex": "3",
                        **_panel_style(),
                        "display": "flex", "flexDirection": "column",
                        "height": "calc(65vh + 37px)", "boxSizing": "border-box",
                    },
                    children=[
                        html.Div(
                            style={"display": "flex", "justifyContent": "space-between",
                                   "marginBottom": "10px", "alignItems": "center"},
                            children=[
                                html.Div("🚨 REAL-TIME THREAT LOG",
                                         style={"fontSize": "14px", "fontWeight": "bold", "color": "#e2e8f0"}),
                                html.Div(id="feed-counter", children="0 Events",
                                         style={"fontSize": "11px", "color": "#00f2ff",
                                                "backgroundColor": "rgba(0,242,255,0.1)",
                                                "padding": "2px 6px", "borderRadius": "10px"}),
                            ]
                        ),
                        html.Div(
                            id="alert-feed-container",
                            style={
                                "flex": "1", "overflowY": "auto",
                                "display": "flex", "flexDirection": "column",
                                "gap": "10px", "paddingRight": "5px",
                            },
                            children=[
                                html.Div("Awaiting target detections...",
                                         style={"color": "#64748b", "fontStyle": "italic",
                                                "textAlign": "center", "marginTop": "20px",
                                                "fontSize": "13px"})
                            ]
                        ),
                    ]
                ),
            ]
        ),

        # ── Telemetry table ───────────────────────────────────────────────────
        html.Div(
            style={"marginTop": "20px", **_panel_style()},
            children=[
                html.Div("📋 TARGET TELEMETRY ROSTER",
                         style={"fontSize": "14px", "fontWeight": "bold",
                                "color": "#e2e8f0", "marginBottom": "12px"}),
                html.Div(id="telemetry-table-container"),
            ]
        ),

        dcc.Interval(id="live-interval", interval=1000, n_intervals=0),
    ]
)


# =============================================================================
# Callbacks
# =============================================================================

@app.callback(
    Output("radar-map",               "figure"),
    Output("kpi-total-alerts",        "children"),
    Output("kpi-critical-alerts",     "children"),
    Output("feed-counter",            "children"),
    Output("alert-feed-container",    "children"),
    Output("telemetry-table-container","children"),
    Input("live-interval",            "n_intervals"),
)
def update_dashboard(n):
    state  = get_state()
    drones = state["drones"]
    alerts = state["alerts"]

    fig = _build_map(drones)

    total_alerts    = len(alerts)
    critical_alerts = sum(1 for a in alerts if a["severity"] == "CRITICAL")
    feed_elements   = _build_feed(alerts)
    telemetry_table = _build_telemetry(drones, alerts)

    return (
        fig,
        str(total_alerts),
        str(critical_alerts),
        f"{total_alerts} Events",
        feed_elements,
        telemetry_table,
    )


# =============================================================================
# Map builder
# =============================================================================

def _build_map(drones: list) -> go.Figure:
    fig = go.Figure()

    for d in drones:
        color = DRONE_COLORS.get(d["id"], "#ffffff")

        # Raw GPS track — faint white, shows noise
        true_path = d.get("true_path", [])
        if len(true_path) > 1:
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lon=[pt[1] for pt in true_path],
                lat=[pt[0] for pt in true_path],
                line=dict(width=1, color="#ffffff"),
                opacity=0.2,
                hoverinfo="none",
                showlegend=False,
            ))

        # Kalman-filtered track — vibrant, solid
        filtered_path = d.get("path", [])
        if len(filtered_path) > 1:
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lon=[pt[1] for pt in filtered_path],
                lat=[pt[0] for pt in filtered_path],
                line=dict(width=2, color=color),
                opacity=0.8,
                hoverinfo="none",
                showlegend=False,
            ))

    # Current position markers
    fig.add_trace(go.Scattermapbox(
        mode="markers+text",
        lon=[d["lon"] for d in drones],
        lat=[d["lat"] for d in drones],
        marker=dict(size=14, color=[DRONE_COLORS.get(d["id"], "#fff") for d in drones]),
        text=[d["id"] for d in drones],
        textposition="top right",
        textfont=dict(color="#ffffff", size=11),
        hoverinfo="text",
        hovertext=[
            f"<b>{d['id']} — {d['name']}</b><br>"
            f"Speed: {d['speed']} m/s  |  Heading: {d['heading']}°<br>"
            f"Altitude: {d['alt']} m<br>"
            f"Profile: {d['behavior_type']}<br>"
            f"Pos: {round(d['lat'],4)}°N, {round(d['lon'],4)}°E"
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


# =============================================================================
# Feed builder
# =============================================================================

def _build_feed(alerts: list) -> list:
    if not alerts:
        return [html.Div(
            "No alerts. All monitored airspace secure.",
            style={"color": "#10b981", "textAlign": "center",
                   "marginTop": "20px", "fontSize": "13px"}
        )]

    SEV_COLORS = {
        "CRITICAL": ("#ff2a2a", "rgba(255,42,42,0.08)",  "rgba(255,42,42,0.4)"),
        "MEDIUM":   ("#ff9f1a", "rgba(255,159,26,0.05)", "rgba(255,159,26,0.4)"),
        "LOW":      ("#fff200", "rgba(255,242,0,0.03)",  "rgba(255,242,0,0.4)"),
    }

    cards = []
    for a in alerts[:50]:
        badge, bg, border = SEV_COLORS.get(a["severity"], SEV_COLORS["LOW"])
        cards.append(html.Div(
            style={
                "backgroundColor": bg,
                "border": f"1px solid {border}",
                "borderRadius": "6px",
                "padding": "10px",
                "display": "flex", "flexDirection": "column", "gap": "5px",
            },
            children=[
                html.Div(
                    style={"display": "flex", "justifyContent": "space-between", "alignItems": "center"},
                    children=[
                        html.Span(a["threat_type"].replace("_", " "),
                                  style={"fontSize": "12px", "fontWeight": "bold", "color": badge}),
                        html.Span(a["timestamp"],
                                  style={"fontSize": "10px", "color": "#64748b"}),
                    ]
                ),
                html.Div(f"Target {a['drone_id']} breached {a['zone_breached']} zone.",
                         style={"fontSize": "12px", "color": "#e2e8f0"}),
                html.Div(
                    f"Speed: {a['speed']} m/s  |  Alt: {a.get('alt','—')} m  |  "
                    f"Pos: {a['lat']}, {a['lon']}",
                    style={"fontSize": "11px", "color": "#94a3b8"}
                ),
            ]
        ))
    return cards


# =============================================================================
# Telemetry table builder
# =============================================================================

def _build_telemetry(drones: list, alerts: list) -> html.Table:
    header = html.Tr(
        style={"borderBottom": "1px solid rgba(255,255,255,0.05)",
               "color": "#94a3b8", "fontSize": "12px", "textAlign": "left"},
        children=[
            html.Th(col, style={"padding": "8px"})
            for col in ["ID", "Callsign", "Profile", "Speed", "Heading", "Altitude", "Coordinates", "Status"]
        ]
    )

    rows = [header]
    for d in drones:
        d_alerts = [a for a in alerts if a["drone_id"] == d["id"]]
        status_text, status_color = "NOMINAL", "#10b981"
        if d_alerts:
            sev = d_alerts[0]["severity"]
            if sev == "CRITICAL":
                status_text, status_color = "CRITICAL INTRUSION", "#ff2a2a"
            elif sev == "MEDIUM":
                status_text, status_color = "WARNING / LOITER",   "#ff9f1a"
            else:
                status_text, status_color = "MONITORED",          "#fff200"

        rows.append(html.Tr(
            style={"borderBottom": "1px solid rgba(255,255,255,0.03)", "fontSize": "13px", "color": "#e2e8f0"},
            children=[
                html.Td(html.Strong(d["id"], style={"color": DRONE_COLORS.get(d["id"], "#fff")}), style={"padding": "8px"}),
                html.Td(d["name"],                          style={"padding": "8px"}),
                html.Td(d["behavior_type"].replace("_"," "),style={"padding": "8px", "color": "#94a3b8"}),
                html.Td(f"{d['speed']} m/s",                style={"padding": "8px"}),
                html.Td(f"{d['heading']}°",                 style={"padding": "8px"}),
                html.Td(f"{d['alt']} m",                    style={"padding": "8px", "color": "#00f2ff"}),
                html.Td(f"{round(d['lat'],4)}, {round(d['lon'],4)}",
                        style={"padding": "8px", "fontFamily": "monospace"}),
                html.Td(
                    html.Span(status_text, style={
                        "color": status_color, "fontWeight": "bold", "fontSize": "11px",
                        "backgroundColor": f"{status_color}15",
                        "padding": "2px 6px", "borderRadius": "4px",
                    }),
                    style={"padding": "8px"}
                ),
            ]
        ))

    return html.Table(
        style={"width": "100%", "borderCollapse": "collapse"},
        children=rows
    )


# =============================================================================
# Helpers (moved above layout — see top of file)
# =============================================================================


# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    if DATA_SOURCE == "simulation":
        simulation_engine.start()
    else:
        adapter.start()
    app.run(debug=False, port=8050, host="0.0.0.0")
