from dash import Dash, html, dcc, Input, Output
import plotly.graph_objects as go
from config import BASE_LAT, BASE_LON, ZONES, METERS_PER_DEGREE
from geofence import geofence_manager
from simulation import simulation_engine

app = Dash(__name__, title="Counter-UAV Geo-fence IDS")

# Distinct vibrant color mapping for premium presentation
DRONE_COLORS = {
    "UAV-01": "#00f2ff", # Cyan Accent
    "UAV-02": "#ff9f1a", # Vibrant Orange
    "UAV-03": "#ff2a2a", # Urgent Red
    "UAV-04": "#bc13fe", # Electric Purple
}

app.layout = html.Div(
    style={
        "backgroundColor": "#0b0f19",
        "color": "#f1f5f9",
        "fontFamily": "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
        "minHeight": "100vh",
        "padding": "20px",
        "boxSizing": "border-box",
        "overflowX": "hidden"
    },
    children=[
        # Top Header Banner
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "borderBottom": "1px solid rgba(255, 255, 255, 0.1)",
                "paddingBottom": "15px",
                "marginBottom": "20px"
            },
            children=[
                html.Div([
                    html.H1(
                        "🛡️ COUNTER-UAV GEO-FENCE IDS",
                        style={
                            "margin": 0,
                            "fontSize": "28px",
                            "fontWeight": "800",
                            "letterSpacing": "1px",
                            "color": "#00f2ff",
                            "textShadow": "0 0 10px rgba(0, 242, 255, 0.3)"
                        }
                    ),
                    html.Div(
                        "Real-time Multi-Zone Aerial Threat Tracking & Kinematics Engine",
                        style={
                            "fontSize": "13px",
                            "color": "#94a3b8",
                            "marginTop": "4px",
                            "textTransform": "uppercase",
                            "letterSpacing": "0.5px"
                        }
                    )
                ]),
                # Live Indicator Badge
                html.Div(
                    style={
                        "display": "flex",
                        "alignItems": "center",
                        "backgroundColor": "rgba(16, 185, 129, 0.1)",
                        "border": "1px solid #10b981",
                        "padding": "6px 12px",
                        "borderRadius": "20px"
                    },
                    children=[
                        html.Div(
                            style={
                                "width": "8px",
                                "height": "8px",
                                "backgroundColor": "#10b981",
                                "borderRadius": "50%",
                                "marginRight": "8px",
                                "boxShadow": "0 0 8px #10b981"
                            }
                        ),
                        html.Span(
                            "SYSTEM LIVE",
                            style={
                                "fontSize": "12px",
                                "fontWeight": "bold",
                                "color": "#10b981",
                                "letterSpacing": "1px"
                            }
                        )
                    ]
                )
            ]
        ),

        # KPI Metrics Row
        html.Div(
            style={
                "display": "flex",
                "gap": "20px",
                "marginBottom": "20px"
            },
            children=[
                # Card 1: Active Tracked Drones
                html.Div(
                    style={
                        "flex": "1",
                        "backgroundColor": "rgba(20, 27, 45, 0.6)",
                        "border": "1px solid rgba(255, 255, 255, 0.08)",
                        "borderRadius": "10px",
                        "padding": "15px",
                        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)"
                    },
                    children=[
                        html.Div("TRACKED AERIAL TARGETS", style={"fontSize": "12px", "color": "#94a3b8", "fontWeight": "600"}),
                        html.Div(id="kpi-active-drones", children="4", style={"fontSize": "28px", "fontWeight": "bold", "color": "#00f2ff", "marginTop": "5px"})
                    ]
                ),
                # Card 2: Total Alerts Triggered
                html.Div(
                    style={
                        "flex": "1",
                        "backgroundColor": "rgba(20, 27, 45, 0.6)",
                        "border": "1px solid rgba(255, 255, 255, 0.08)",
                        "borderRadius": "10px",
                        "padding": "15px",
                        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)"
                    },
                    children=[
                        html.Div("TOTAL ALERTS GENERATED", style={"fontSize": "12px", "color": "#94a3b8", "fontWeight": "600"}),
                        html.Div(id="kpi-total-alerts", children="0", style={"fontSize": "28px", "fontWeight": "bold", "color": "#ff9f1a", "marginTop": "5px"})
                    ]
                ),
                # Card 3: Critical Incidents
                html.Div(
                    style={
                        "flex": "1",
                        "backgroundColor": "rgba(20, 27, 45, 0.6)",
                        "border": "1px solid rgba(255, 255, 255, 0.08)",
                        "borderRadius": "10px",
                        "padding": "15px",
                        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)"
                    },
                    children=[
                        html.Div("CRITICAL EXCLUSION BREACHES", style={"fontSize": "12px", "color": "#94a3b8", "fontWeight": "600"}),
                        html.Div(id="kpi-critical-alerts", children="0", style={"fontSize": "28px", "fontWeight": "bold", "color": "#ff2a2a", "marginTop": "5px"})
                    ]
                ),
            ]
        ),

        # Main Workspace: Map (Left 70%) + Scrolling Threat Feed (Right 30%)
        html.Div(
            style={
                "display": "flex",
                "gap": "20px",
                "alignItems": "stretch"
            },
            children=[
                # Left Panel: Live Scattermapbox Container
                html.Div(
                    style={
                        "flex": "7",
                        "backgroundColor": "rgba(20, 27, 45, 0.6)",
                        "border": "1px solid rgba(255, 255, 255, 0.08)",
                        "borderRadius": "10px",
                        "padding": "15px",
                        "display": "flex",
                        "flexDirection": "column",
                        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)"
                    },
                    children=[
                        html.Div(
                            style={"display": "flex", "justifyContent": "space-between", "marginBottom": "10px", "alignItems": "center"},
                            children=[
                                html.Div("🛰️ TACTICAL RADAR VIEW", style={"fontSize": "14px", "fontWeight": "bold", "color": "#e2e8f0"}),
                                html.Div("Center: Bengaluru (12.97°N, 77.59°E)", style={"fontSize": "11px", "color": "#64748b"})
                            ]
                        ),
                        dcc.Graph(
                            id="radar-map",
                            style={"height": "65vh", "width": "100%"},
                            config={"displayModeBar": False}
                        )
                    ]
                ),

                # Right Panel: Live Scrolling Threat Feed
                html.Div(
                    style={
                        "flex": "3",
                        "backgroundColor": "rgba(20, 27, 45, 0.6)",
                        "border": "1px solid rgba(255, 255, 255, 0.08)",
                        "borderRadius": "10px",
                        "padding": "15px",
                        "display": "flex",
                        "flexDirection": "column",
                        "boxShadow": "0 4px 20px rgba(0,0,0,0.3)",
                        "height": "calc(65vh + 37px)",
                        "boxSizing": "border-box"
                    },
                    children=[
                        html.Div(
                            style={"display": "flex", "justifyContent": "space-between", "marginBottom": "10px", "alignItems": "center"},
                            children=[
                                html.Div("🚨 REAL-TIME THREAT LOG", style={"fontSize": "14px", "fontWeight": "bold", "color": "#e2e8f0"}),
                                html.Div(id="feed-counter", children="0 Events", style={"fontSize": "11px", "color": "#00f2ff", "backgroundColor": "rgba(0, 242, 255, 0.1)", "padding": "2px 6px", "borderRadius": "10px"})
                            ]
                        ),
                        # Scrollable Alert Feed Container
                        html.Div(
                            id="alert-feed-container",
                            style={
                                "flex": "1",
                                "overflowY": "auto",
                                "display": "flex",
                                "flexDirection": "column",
                                "gap": "10px",
                                "paddingRight": "5px"
                            },
                            children=[
                                html.Div(
                                    "Awaiting target detections...",
                                    style={"color": "#64748b", "fontStyle": "italic", "textAlign": "center", "marginTop": "20px", "fontSize": "13px"}
                                )
                            ]
                        )
                    ]
                )
            ]
        ),

        # Drones Info List / State Table Row below map
        html.Div(
            style={
                "marginTop": "20px",
                "backgroundColor": "rgba(20, 27, 45, 0.6)",
                "border": "1px solid rgba(255, 255, 255, 0.08)",
                "borderRadius": "10px",
                "padding": "15px",
                "boxShadow": "0 4px 20px rgba(0,0,0,0.3)"
            },
            children=[
                html.Div("📋 TARGET TELEMETRY ROSTER", style={"fontSize": "14px", "fontWeight": "bold", "color": "#e2e8f0", "marginBottom": "12px"}),
                html.Div(id="telemetry-table-container")
            ]
        ),

        # Polling Interval: Polling reactive update state every 1.0 second
        dcc.Interval(id="live-interval", interval=1000, n_intervals=0)
    ]
)

@app.callback(
    Output("radar-map", "figure"),
    Output("kpi-total-alerts", "children"),
    Output("kpi-critical-alerts", "children"),
    Output("feed-counter", "children"),
    Output("alert-feed-container", "children"),
    Output("telemetry-table-container", "children"),
    Input("live-interval", "n_intervals")
)
def update_dashboard(n):
    state = simulation_engine.get_state()
    drones = state["drones"]
    alerts = state["alerts"]

    # 1. Build Tactically Accurate Mapbox Figure
    fig = go.Figure()

    # Add Trajectory Tails
    for d in drones:
        path = d["path"]
        if len(path) > 1:
            lats = [pt[0] for pt in path]
            lons = [pt[1] for pt in path]
            color = DRONE_COLORS.get(d["id"], "#ffffff")
            fig.add_trace(go.Scattermapbox(
                mode="lines",
                lon=lons,
                lat=lats,
                line=dict(width=2, color=color),
                opacity=0.6,
                hoverinfo="none",
                showlegend=False
            ))

    # Add Kalman predicted trajectory + uncertainty cone
    for d in drones:
        predicted = d.get("predicted_path", [])
        if not predicted:
            continue

        pred_lats = []
        pred_lons = []
        cone_lats = []
        cone_lons = []

        for (px, py, sigma) in predicted:
            # Convert meters back to lat/lon
            lat = BASE_LAT + (py / METERS_PER_DEGREE)
            lon = BASE_LON + (px / METERS_PER_DEGREE)
            pred_lats.append(lat)
            pred_lons.append(lon)

            # Uncertainty radius in degrees
            sigma_deg = sigma / METERS_PER_DEGREE
            cone_lats.append(lat)
            cone_lons.append(lon)

        color = DRONE_COLORS.get(d["id"], "#ffffff")

        # Predicted path line — dashed look via opacity
        fig.add_trace(go.Scattermapbox(
            mode="lines+markers",
            lon=pred_lons,
            lat=pred_lats,
            line=dict(width=1, color=color),
            marker=dict(size=4, color=color, opacity=0.4),
            opacity=0.35,
            hoverinfo="none",
            showlegend=False
        ))

    # Add Live Target Markers
    curr_lats = [d["lat"] for d in drones]
    curr_lons = [d["lon"] for d in drones]
    curr_texts = [f"<b>{d['id']} ({d['name']})</b><br>Speed: {d['speed']} m/s<br>Heading: {d['heading']}°<br>Alt: {d['alt']}m<br>Profile: {d['behavior_type']}" for d in drones]
    curr_colors = [DRONE_COLORS.get(d["id"], "#ffffff") for d in drones]

    fig.add_trace(go.Scattermapbox(
        mode="markers+text",
        lon=curr_lons,
        lat=curr_lats,
        marker=dict(
            size=14,
            color=curr_colors,
            opacity=1.0,
        ),
        text=[d["id"] for d in drones],
        textposition="top right",
        textfont=dict(color="#ffffff", size=11, family="sans-serif"),
        hoverinfo="text",
        hovertext=curr_texts,
        showlegend=False
    ))

    # Configure Map Layout with Multi-Zone GeoJSON Layers
    fig.update_layout(
        mapbox=dict(
            style="carto-darkmatter",
            center=dict(lat=BASE_LAT, lon=BASE_LON),
            zoom=12.2,
            layers=geofence_manager.get_mapbox_layers()
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        uirevision="constant" # Secures dynamic user zoom/pan consistency across frame refreshes
    )

    # 2. Compute Top Line KPIs
    total_alerts = len(alerts)
    critical_alerts = sum(1 for a in alerts if a["severity"] == "CRITICAL")

    # 3. Render Alert Logs
    feed_elements = []
    if not alerts:
        feed_elements.append(html.Div(
            "No alerts detected yet. All monitored airspace secure.",
            style={"color": "#10b981", "textAlign": "center", "marginTop": "20px", "fontSize": "13px"}
        ))
    else:
        # Display bounded visual sequence of modern styled alert entries
        for a in alerts[:50]:
            sev = a["severity"]
            if sev == "CRITICAL":
                border_color = "rgba(255, 42, 42, 0.4)"
                bg_color = "rgba(255, 42, 42, 0.08)"
                badge_color = "#ff2a2a"
            elif sev == "MEDIUM":
                border_color = "rgba(255, 159, 26, 0.4)"
                bg_color = "rgba(255, 159, 26, 0.05)"
                badge_color = "#ff9f1a"
            else:
                border_color = "rgba(255, 242, 0, 0.4)"
                bg_color = "rgba(255, 242, 0, 0.03)"
                badge_color = "#fff200"

            card = html.Div(
                style={
                    "backgroundColor": bg_color,
                    "border": f"1px solid {border_color}",
                    "borderRadius": "6px",
                    "padding": "10px",
                    "display": "flex",
                    "flexDirection": "column",
                    "gap": "5px"
                },
                children=[
                    html.Div(
                        style={"display": "flex", "justifyContent": "space-between", "alignItems": "center"},
                        children=[
                            html.Span(a["threat_type"].replace("_", " "), style={"fontSize": "12px", "fontWeight": "bold", "color": badge_color}),
                            html.Span(a["timestamp"], style={"fontSize": "10px", "color": "#64748b"})
                        ]
                    ),
                    html.Div(f"Target {a['drone_id']} breached {a['zone_breached']} zone.", style={"fontSize": "12px", "color": "#e2e8f0"}),
                    html.Div(f"Speed: {a['speed']} m/s | Pos: {a['lat']}, {a['lon']}", style={"fontSize": "11px", "color": "#94a3b8"})
                ]
            )
            feed_elements.append(card)

    # 4. Generate Responsive Telemetry Roster
    table_rows = [
        html.Tr(
            style={"borderBottom": "1px solid rgba(255, 255, 255, 0.05)", "color": "#94a3b8", "fontSize": "12px", "textAlign": "left"},
            children=[
                html.Th("ID", style={"padding": "8px"}),
                html.Th("Callsign", style={"padding": "8px"}),
                html.Th("Assigned Profile", style={"padding": "8px"}),
                html.Th("Speed", style={"padding": "8px"}),
                html.Th("Heading", style={"padding": "8px"}),
                html.Th("Coordinates", style={"padding": "8px"}),
                html.Th("Status", style={"padding": "8px"})
            ]
        )
    ]

    for d in drones:
        d_alerts = [a for a in alerts if a["drone_id"] == d["id"]]
        status_text = "NORMAL"
        status_color = "#10b981"
        if d_alerts:
            latest_sev = d_alerts[0]["severity"]
            if latest_sev == "CRITICAL":
                status_text = "CRITICAL INTRUSION"
                status_color = "#ff2a2a"
            elif latest_sev == "MEDIUM":
                status_text = "WARNING / LOITER"
                status_color = "#ff9f1a"
            else:
                status_text = "MONITORED"
                status_color = "#fff200"

        row = html.Tr(
            style={"borderBottom": "1px solid rgba(255, 255, 255, 0.03)", "fontSize": "13px", "color": "#e2e8f0"},
            children=[
                html.Td(html.Strong(d["id"], style={"color": DRONE_COLORS.get(d["id"], "#fff")}), style={"padding": "8px"}),
                html.Td(d["name"], style={"padding": "8px"}),
                html.Td(d["behavior_type"].replace("_", " "), style={"padding": "8px", "color": "#94a3b8"}),
                html.Td(f"{d['speed']} m/s", style={"padding": "8px"}),
                html.Td(f"{d['heading']}°", style={"padding": "8px"}),
                html.Td(f"{round(d['lat'], 4)}, {round(d['lon'], 4)}", style={"padding": "8px", "fontFamily": "monospace"}),
                html.Td(html.Span(status_text, style={"color": status_color, "fontWeight": "bold", "fontSize": "11px", "backgroundColor": f"{status_color}15", "padding": "2px 6px", "borderRadius": "4px"}), style={"padding": "8px"})
            ]
        )
        table_rows.append(row)

    telemetry_table = html.Table(
        style={"width": "100%", "borderCollapse": "collapse"},
        children=table_rows
    )

    feed_counter_text = f"{len(alerts)} Events"

    return fig, str(total_alerts), str(critical_alerts), feed_counter_text, feed_elements, telemetry_table

# Initialize the simulation thread globally so it always runs
simulation_engine.start()

if __name__ == "__main__":
    app.run(debug=False, port=8050, host="0.0.0.0")
