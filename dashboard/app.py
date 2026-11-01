
import streamlit as st
import pandas as pd

from data_loader import (
    load_all_data,
    get_data_summary,
    get_result_images,
    PHASE5_DIR
)

# --------------------------------------------------
# PAGE CONFIGURATION
# --------------------------------------------------

st.set_page_config(
    page_title="VANET Analytics",
    page_icon="🚗",
    layout="wide",
    initial_sidebar_state="expanded"
)

# --------------------------------------------------
# CUSTOM CSS
# --------------------------------------------------

st.markdown("""
<style>
    .stApp {
        background-color: #0E1117;
        color: #F5F5F5;
    }

    [data-testid="stSidebar"] {
        background-color: #151A23;
        border-right: 1px solid #293241;
    }

    .main-title {
        font-size: 34px;
        font-weight: 750;
        color: #F5F5F5;
        margin-bottom: 0;
    }

    .sub-title {
        color: #A0AEC0;
        font-size: 15px;
        margin-top: 5px;
        margin-bottom: 25px;
    }

    .section-title {
        font-size: 21px;
        font-weight: 650;
        margin-top: 20px;
        margin-bottom: 12px;
    }

    div[data-testid="stMetric"] {
        background-color: #171E2B;
        border: 1px solid #29364A;
        padding: 18px;
        border-radius: 12px;
    }

    div[data-testid="stMetricValue"] {
        color: #60A5FA;
    }

    .info-card {
        background-color: #171E2B;
        border: 1px solid #29364A;
        padding: 18px;
        border-radius: 12px;
        min-height: 115px;
    }

    .info-card h4 {
        color: #60A5FA;
        margin-bottom: 8px;
    }

    .info-card p {
        color: #CBD5E1;
        font-size: 14px;
    }

    hr {
        border-color: #29364A;
    }
</style>
""", unsafe_allow_html=True)


# --------------------------------------------------
# LOAD PROJECT DATA
# --------------------------------------------------

data = load_all_data()
summary = get_data_summary(data)


# --------------------------------------------------
# SIDEBAR
# --------------------------------------------------

st.sidebar.title("🚗 VANET Analytics")
st.sidebar.caption("Project Results Dashboard")

st.sidebar.markdown("---")

page = st.sidebar.radio(
    "NAVIGATION",
    [
        "Project Overview",
        "Vehicle Simulation",
        "Collision Detection",
        "Scenario Analysis",
        "Network Performance",
        "Results Gallery"
    ]
)

st.sidebar.markdown("---")

st.sidebar.markdown("### Project Information")
st.sidebar.caption(
    "VANET-Based Collision Warning "
    "and Traffic Management Using AODV Routing"
)

st.sidebar.markdown("---")
st.sidebar.caption("Review 1 | Simulation Results")


# --------------------------------------------------
# PAGE HEADER
# --------------------------------------------------

st.markdown(
    '<div class="main-title">VANET Analytics</div>',
    unsafe_allow_html=True
)

st.markdown(
    '<div class="sub-title">'
    'Collision Warning and Traffic Management Using AODV'
    '</div>',
    unsafe_allow_html=True
)


# ==================================================
# PAGE 1: PROJECT OVERVIEW
# ==================================================

if page == "Project Overview":

    st.markdown("## Project Overview")

    st.write(
        "An integrated vehicular network simulation designed "
        "to study vehicle mobility, wireless communication, "
        "collision detection and warning generation."
    )

    # KPI CARDS

    col1, col2, col3, col4 = st.columns(4)

    with col1:
        st.metric(
            "Simulated Vehicles",
            summary["vehicles"]
        )

    with col2:
        st.metric(
            "Test Scenarios",
            summary["scenarios"]
        )

    with col3:
        st.metric(
            "Warnings Generated",
            summary["warnings_generated"]
        )

    with col4:
        st.metric(
            "Warnings Delivered",
            summary["warnings_delivered"]
        )

    st.markdown("---")

    # ARCHITECTURE

    st.markdown("## System Architecture")

    st.markdown("""
    <div class="info-card">
        <h4>1. SUMO – Traffic Simulation</h4>
        <p>Simulates road networks, vehicle movement,
        speed and traffic conditions.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("""
    <div class="info-card">
        <h4>2. Veins – Integration Layer</h4>
        <p>Connects SUMO vehicle mobility with
        the OMNeT++ network simulation.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("""
    <div class="info-card">
        <h4>3. OMNeT++ and INET – Network Simulation</h4>
        <p>Provides the wireless communication
        and network protocol simulation environment.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("""
    <div class="info-card">
        <h4>4. AODV – Routing Protocol</h4>
        <p>Discovers routes between vehicles
        when communication is required.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("""
    <div class="info-card">
        <h4>5. Collision Detection</h4>
        <p>Uses vehicle distance, relative speed
        and TTC to identify potential collision risks
        and generate warning messages.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("---")

    # DATASET SUMMARY

    st.markdown("## Available Simulation Data")

    dataset_info = []

    for name, df in data.items():
        dataset_info.append({
            "Dataset": name.replace("_", " ").title(),
            "Records": len(df),
            "Columns": len(df.columns),
            "Status": "Available" if not df.empty else "Empty"
        })

    st.dataframe(
        pd.DataFrame(dataset_info),
        use_container_width=True,
        hide_index=True
    )

    # DATA CONSISTENCY NOTE

    transmission_records = len(data["warning_transmission"])

    st.info(
        f"The scenario evaluation reports "
        f"{summary['warnings_generated']} generated warnings and "
        f"{summary['warnings_delivered']} delivered warnings. "
        f"The detailed warning transmission log currently contains "
        f"{transmission_records} records. These are different "
        f"data sources and are displayed separately."
    )


# ==================================================
# PAGE 2: PLACEHOLDER
# ==================================================

elif page == "Vehicle Simulation":

    import plotly.express as px

    st.header("Vehicle Simulation")

    st.caption(
        "Analysis of vehicle mobility recorded by SUMO "
        "during the traffic simulation."
    )

    df = data["vehicle_state"].copy()

    if df.empty:
        st.warning("Vehicle state data is unavailable.")

    else:

        # Ensure numeric columns are numeric
        numeric_cols = [
            "timestamp", "pos_x", "pos_y",
            "speed_mps", "speed_kmh"
        ]

        for col in numeric_cols:
            if col in df.columns:
                df[col] = pd.to_numeric(
                    df[col], errors="coerce"
                )

        df = df.dropna(
            subset=["timestamp", "vehicle_id"]
        )

        vehicles = sorted(
            df["vehicle_id"].dropna().unique()
        )

        # Vehicle selector
        selected_vehicles = st.multiselect(
            "Select vehicles",
            vehicles,
            default=vehicles
        )

        filtered = df[
            df["vehicle_id"].isin(selected_vehicles)
        ].copy()

        if filtered.empty:
            st.warning("Select at least one vehicle.")

        else:

            # ----------------------------------
            # VEHICLE KPIs
            # ----------------------------------

            col1, col2, col3 = st.columns(3)

            with col1:
                st.metric(
                    "Vehicles Selected",
                    filtered["vehicle_id"].nunique()
                )

            with col2:
                st.metric(
                    "Average Speed",
                    f"{filtered['speed_kmh'].mean():.2f} km/h"
                )

            with col3:
                st.metric(
                    "Maximum Speed",
                    f"{filtered['speed_kmh'].max():.2f} km/h"
                )

            st.markdown("---")

            # ----------------------------------
            # SPEED OVER TIME
            # ----------------------------------

            st.subheader("Vehicle Speed Over Time")

            speed_fig = px.line(
                filtered,
                x="timestamp",
                y="speed_kmh",
                color="vehicle_id",
                title="Speed Variation Across Vehicles",
                labels={
                    "timestamp": "Simulation Time (s)",
                    "speed_kmh": "Speed (km/h)",
                    "vehicle_id": "Vehicle"
                },
                template="plotly_dark"
            )

            speed_fig.update_layout(
                hovermode="x unified",
                legend_title="Vehicle ID"
            )

            st.plotly_chart(
                speed_fig,
                use_container_width=True
            )

            # ----------------------------------
            # POSITION OVER TIME
            # ----------------------------------

            st.subheader("Vehicle Position Over Time")

            position_fig = px.line(
                filtered,
                x="timestamp",
                y="pos_x",
                color="vehicle_id",
                title="Longitudinal Vehicle Position",
                labels={
                    "timestamp": "Simulation Time (s)",
                    "pos_x": "Position (m)",
                    "vehicle_id": "Vehicle"
                },
                template="plotly_dark"
            )

            position_fig.update_layout(
                hovermode="x unified"
            )

            st.plotly_chart(
                position_fig,
                use_container_width=True
            )

            # ----------------------------------
            # SPEED COMPARISON
            # ----------------------------------

            st.subheader("Average Speed by Vehicle")

            average_speed = (
                filtered.groupby("vehicle_id")["speed_kmh"]
                .mean()
                .reset_index()
            )

            average_fig = px.bar(
                average_speed,
                x="vehicle_id",
                y="speed_kmh",
                color="vehicle_id",
                title="Average Speed Comparison",
                labels={
                    "vehicle_id": "Vehicle",
                    "speed_kmh": "Average Speed (km/h)"
                },
                template="plotly_dark"
            )

            st.plotly_chart(
                average_fig,
                use_container_width=True
            )

            # ----------------------------------
            # RAW DATA
            # ----------------------------------

            with st.expander("View Vehicle Telemetry Data"):

                st.dataframe(
                    filtered,
                    use_container_width=True,
                    hide_index=True
                )

                st.download_button(
                    "Download Vehicle Data",
                    data=filtered.to_csv(index=False),
                    file_name="vehicle_simulation_data.csv",
                    mime="text/csv"
                )



# ==================================================
# PAGE 3: PLACEHOLDER
# ==================================================

elif page == "Collision Detection":

    import plotly.express as px

    st.header("Collision Detection Analysis")

    st.caption(
        "Analysis of vehicle proximity, relative speed, "
        "Time-to-Collision and generated collision warnings."
    )

    df = data["collision_detection"].copy()

    if df.empty:
        st.warning("Collision detection data is unavailable.")

    else:

        # ----------------------------------
        # DATA PREPARATION
        # ----------------------------------

        numeric_cols = [
            "timestamp",
            "gap_m",
            "closing_speed_mps",
            "ttc_s"
        ]

        for col in numeric_cols:
            df[col] = pd.to_numeric(
                df[col], errors="coerce"
            )

        df["risk_level"] = (
            df["risk_level"]
            .fillna("UNKNOWN")
            .astype(str)
            .str.upper()
        )

        df["warning_generated"] = (
            df["warning_generated"]
            .astype(str)
            .str.lower()
            .isin(["true", "1", "yes"])
        )

        # ----------------------------------
        # FILTERS
        # ----------------------------------

        st.subheader("Analysis Filters")

        col1, col2 = st.columns(2)

        with col1:
            vehicles = sorted(
                set(df["follower_id"].dropna()) |
                set(df["leader_id"].dropna())
            )

            selected_vehicles = st.multiselect(
                "Select vehicles",
                vehicles,
                default=vehicles
            )

        with col2:
            risk_levels = sorted(
                df["risk_level"].unique()
            )

            selected_risks = st.multiselect(
                "Select risk levels",
                risk_levels,
                default=risk_levels
            )

        filtered = df[
            (
                df["follower_id"].isin(selected_vehicles) |
                df["leader_id"].isin(selected_vehicles)
            ) &
            df["risk_level"].isin(selected_risks)
        ].copy()

        if filtered.empty:
            st.warning("No records match the selected filters.")

        else:

            # ----------------------------------
            # KPI CARDS
            # ----------------------------------

            total_records = len(filtered)

            warnings = int(
                filtered["warning_generated"].sum()
            )

            critical = int(
                (filtered["risk_level"] == "CRITICAL").sum()
            )

            avg_ttc = filtered["ttc_s"].replace(
                [float("inf"), -float("inf")],
                float("nan")
            ).mean()

            col1, col2, col3, col4 = st.columns(4)

            col1.metric("Vehicle Pair Records", total_records)
            col2.metric("Warnings Generated", warnings)
            col3.metric("Critical Risk Records", critical)
            col4.metric(
                "Average TTC",
                f"{avg_ttc:.2f} s"
                if pd.notna(avg_ttc) else "N/A"
            )

            st.markdown("---")

            # ----------------------------------
            # TTC OVER TIME
            # ----------------------------------

            st.subheader("Time-to-Collision Dynamics")

            ttc_data = filtered[
                filtered["ttc_s"].notna() &
                (filtered["ttc_s"] >= 0)
            ].copy()

            if not ttc_data.empty:

                ttc_fig = px.line(
                    ttc_data,
                    x="timestamp",
                    y="ttc_s",
                    color="follower_id",
                    line_dash="risk_level",
                    markers=True,
                    title="TTC Variation Over Simulation Time",
                    labels={
                        "timestamp": "Simulation Time (s)",
                        "ttc_s": "Time-to-Collision (seconds)",
                        "follower_id": "Following Vehicle",
                        "risk_level": "Risk Level"
                    },
                    template="plotly_dark"
                )

                ttc_fig.update_layout(
                    hovermode="x unified"
                )

                st.plotly_chart(
                    ttc_fig,
                    use_container_width=True
                )

            else:
                st.info("No valid TTC values available.")

            # ----------------------------------
            # GAP AND CLOSING SPEED
            # ----------------------------------

            col1, col2 = st.columns(2)

            with col1:

                st.subheader("Vehicle Gap")

                gap_fig = px.scatter(
                    filtered,
                    x="timestamp",
                    y="gap_m",
                    color="risk_level",
                    hover_data=[
                        "follower_id",
                        "leader_id"
                    ],
                    title="Bumper-to-Bumper Gap",
                    labels={
                        "timestamp": "Time (s)",
                        "gap_m": "Gap (metres)"
                    },
                    template="plotly_dark"
                )

                st.plotly_chart(
                    gap_fig,
                    use_container_width=True
                )

            with col2:

                st.subheader("Relative Closing Speed")

                speed_fig = px.scatter(
                    filtered,
                    x="timestamp",
                    y="closing_speed_mps",
                    color="risk_level",
                    hover_data=[
                        "follower_id",
                        "leader_id"
                    ],
                    title="Closing Speed Over Time",
                    labels={
                        "timestamp": "Time (s)",
                        "closing_speed_mps": "Closing Speed (m/s)"
                    },
                    template="plotly_dark"
                )

                st.plotly_chart(
                    speed_fig,
                    use_container_width=True
                )

            st.markdown("---")

            # ----------------------------------
            # RISK DISTRIBUTION
            # ----------------------------------

            st.subheader("Risk Level Distribution")

            risk_counts = (
                filtered["risk_level"]
                .value_counts()
                .reset_index()
            )

            risk_counts.columns = [
                "Risk Level",
                "Records"
            ]

            risk_fig = px.pie(
                risk_counts,
                names="Risk Level",
                values="Records",
                hole=0.5,
                title="Distribution of Collision Risk Levels",
                template="plotly_dark"
            )

            st.plotly_chart(
                risk_fig,
                use_container_width=True
            )

            st.markdown("---")

            # ----------------------------------
            # WARNING EVENTS
            # ----------------------------------

            st.subheader("Generated Warning Events")

            warning_events = filtered[
                filtered["warning_generated"]
            ]

            if not warning_events.empty:

                st.dataframe(
                    warning_events,
                    use_container_width=True,
                    hide_index=True
                )

            else:
                st.info(
                    "No warnings were generated "
                    "for the selected filters."
                )

            # ----------------------------------
            # DOWNLOAD
            # ----------------------------------

            st.download_button(
                "Download Collision Detection Data",
                data=filtered.to_csv(index=False),
                file_name="collision_detection_analysis.csv",
                mime="text/csv"
            )

            with st.expander("View All Collision Detection Records"):

                st.dataframe(
                    filtered,
                    use_container_width=True,
                    hide_index=True
                )



# ==================================================
# PAGE 4: PLACEHOLDER
# ==================================================

elif page == "Scenario Analysis":

    import plotly.express as px
    import plotly.graph_objects as go

    st.header("Scenario Analysis")

    st.caption(
        "Comparative evaluation of collision detection "
        "across six simulated traffic scenarios."
    )

    df = data["scenario_metrics"].copy()

    if df.empty:
        st.warning("Scenario evaluation data is unavailable.")

    else:

        # ----------------------------------
        # DATA PREPARATION
        # ----------------------------------

        numeric_cols = [
            "TP", "FP", "TN", "FN",
            "Precision", "Recall", "F1",
            "FalseAlarmRate", "MeanLatency_s",
            "WarningsGenerated", "WarningsDelivered"
        ]

        for col in numeric_cols:
            if col in df.columns:
                df[col] = pd.to_numeric(
                    df[col], errors="coerce"
                )

        # Calculate metrics from confusion matrix counts.
        # This avoids treating undefined metrics as 100%.

        def safe_divide(numerator, denominator):
            if pd.isna(denominator) or denominator == 0:
                return float("nan")
            return numerator / denominator

        df["Precision_Display"] = df.apply(
            lambda r: safe_divide(
                r["TP"], r["TP"] + r["FP"]
            ),
            axis=1
        )

        df["Recall_Display"] = df.apply(
            lambda r: safe_divide(
                r["TP"], r["TP"] + r["FN"]
            ),
            axis=1
        )

        df["F1_Display"] = df.apply(
            lambda r: safe_divide(
                2 * r["TP"],
                2 * r["TP"] + r["FP"] + r["FN"]
            ),
            axis=1
        )

        df["FalseAlarm_Display"] = df.apply(
            lambda r: safe_divide(
                r["FP"], r["FP"] + r["TN"]
            ),
            axis=1
        )

        # ----------------------------------
        # FILTER
        # ----------------------------------

        st.subheader("Scenario Filters")

        scenarios = sorted(df["Scenario"].dropna().unique())

        selected_scenarios = st.multiselect(
            "Select scenarios",
            scenarios,
            default=scenarios
        )

        filtered = df[
            df["Scenario"].isin(selected_scenarios)
        ].copy()

        if filtered.empty:
            st.warning("Select at least one scenario.")

        else:

            # ----------------------------------
            # OVERVIEW KPIs
            # ----------------------------------

            total_tp = int(filtered["TP"].sum())
            total_fp = int(filtered["FP"].sum())
            total_tn = int(filtered["TN"].sum())
            total_fn = int(filtered["FN"].sum())

            total_generated = int(
                filtered["WarningsGenerated"].sum()
            )

            total_delivered = int(
                filtered["WarningsDelivered"].sum()
            )

            overall_precision = safe_divide(
                total_tp, total_tp + total_fp
            )

            overall_recall = safe_divide(
                total_tp, total_tp + total_fn
            )

            overall_f1 = safe_divide(
                2 * total_tp,
                2 * total_tp + total_fp + total_fn
            )

            st.subheader("Overall Evaluation")

            c1, c2, c3, c4 = st.columns(4)

            c1.metric("True Positives", total_tp)
            c2.metric("False Positives", total_fp)
            c3.metric("True Negatives", total_tn)
            c4.metric("False Negatives", total_fn)

            c1, c2, c3 = st.columns(3)

            c1.metric(
                "Overall Precision",
                f"{overall_precision:.1%}"
                if pd.notna(overall_precision) else "N/A"
            )

            c2.metric(
                "Overall Recall",
                f"{overall_recall:.1%}"
                if pd.notna(overall_recall) else "N/A"
            )

            c3.metric(
                "Overall F1 Score",
                f"{overall_f1:.1%}"
                if pd.notna(overall_f1) else "N/A"
            )

            st.markdown("---")

            # ----------------------------------
            # PRECISION RECALL F1 COMPARISON
            # ----------------------------------

            st.subheader("Detection Performance Comparison")

            performance = filtered.melt(
                id_vars="Scenario",
                value_vars=[
                    "Precision_Display",
                    "Recall_Display",
                    "F1_Display"
                ],
                var_name="Metric",
                value_name="Score"
            )

            performance["Metric"] = (
                performance["Metric"]
                .str.replace("_Display", "", regex=False)
            )

            performance["Score_Percent"] = (
                performance["Score"] * 100
            )

            fig = px.bar(
                performance,
                x="Scenario",
                y="Score_Percent",
                color="Metric",
                barmode="group",
                text="Score_Percent",
                title="Precision, Recall and F1 Score by Scenario",
                labels={
                    "Scenario": "Traffic Scenario",
                    "Score_Percent": "Score (%)",
                    "Metric": "Evaluation Metric"
                },
                template="plotly_dark"
            )

            fig.update_traces(
                texttemplate="%{text:.1f}%",
                textposition="outside"
            )

            fig.update_layout(
                yaxis_range=[0, 115],
                xaxis_tickangle=-25
            )

            st.plotly_chart(
                fig,
                use_container_width=True
            )

            st.markdown("---")

            # ----------------------------------
            # WARNING COMPARISON
            # ----------------------------------

            st.subheader("Warning Generation and Delivery")

            warning_data = filtered.melt(
                id_vars="Scenario",
                value_vars=[
                    "WarningsGenerated",
                    "WarningsDelivered"
                ],
                var_name="Category",
                value_name="Count"
            )

            warning_data["Category"] = (
                warning_data["Category"]
                .map({
                    "WarningsGenerated": "Generated",
                    "WarningsDelivered": "Delivered"
                })
            )

            warning_fig = px.bar(
                warning_data,
                x="Scenario",
                y="Count",
                color="Category",
                barmode="group",
                text="Count",
                title="Warnings Generated vs Reported Delivered",
                labels={
                    "Scenario": "Traffic Scenario",
                    "Count": "Number of Warnings",
                    "Category": "Warning Status"
                },
                template="plotly_dark"
            )

            warning_fig.update_layout(
                xaxis_tickangle=-25
            )

            st.plotly_chart(
                warning_fig,
                use_container_width=True
            )

            # ----------------------------------
            # FALSE ALARM RATE
            # ----------------------------------

            st.subheader("False Alarm Rate")

            far_fig = px.bar(
                filtered,
                x="Scenario",
                y="FalseAlarm_Display",
                title="False Alarm Rate by Scenario",
                labels={
                    "Scenario": "Traffic Scenario",
                    "FalseAlarm_Display": "False Alarm Rate"
                },
                template="plotly_dark"
            )

            far_fig.update_layout(
                yaxis_tickformat=".0%",
                xaxis_tickangle=-25
            )

            st.plotly_chart(
                far_fig,
                use_container_width=True
            )

            # ----------------------------------
            # LATENCY
            # ----------------------------------

            st.subheader("Detection Latency")

            latency_fig = px.bar(
                filtered,
                x="Scenario",
                y="MeanLatency_s",
                color="Scenario",
                title="Mean Detection Latency",
                labels={
                    "Scenario": "Traffic Scenario",
                    "MeanLatency_s": "Latency (seconds)"
                },
                template="plotly_dark"
            )

            st.plotly_chart(
                latency_fig,
                use_container_width=True
            )

            st.markdown("---")

            # ----------------------------------
            # DETAILED RESULTS TABLE
            # ----------------------------------

            st.subheader("Detailed Scenario Results")

            display_columns = [
                "Scenario",
                "TP", "FP", "TN", "FN",
                "Precision_Display",
                "Recall_Display",
                "F1_Display",
                "FalseAlarm_Display",
                "MeanLatency_s",
                "WarningsGenerated",
                "WarningsDelivered"
            ]

            display_df = filtered[
                display_columns
            ].copy()

            display_df = display_df.rename(
                columns={
                    "Precision_Display": "Precision",
                    "Recall_Display": "Recall",
                    "F1_Display": "F1 Score",
                    "FalseAlarm_Display": "False Alarm Rate",
                    "MeanLatency_s": "Mean Latency (s)",
                    "WarningsGenerated": "Warnings Generated",
                    "WarningsDelivered": "Warnings Delivered"
                }
            )

            st.dataframe(
                display_df.style.format({
                    "Precision": "{:.2%}",
                    "Recall": "{:.2%}",
                    "F1 Score": "{:.2%}",
                    "False Alarm Rate": "{:.2%}",
                    "Mean Latency (s)": "{:.3f}"
                }, na_rep="N/A"),
                use_container_width=True,
                hide_index=True
            )

            st.download_button(
                "Download Scenario Evaluation",
                data=display_df.to_csv(index=False),
                file_name="scenario_evaluation.csv",
                mime="text/csv"
            )



# ==================================================
# PAGE 5: PLACEHOLDER
# ==================================================

elif page == "Network Performance":

    import plotly.express as px
    import plotly.graph_objects as go

    st.header("Network Performance")

    st.caption(
        "Analysis of AODV routing performance, packet delivery "
        "and collision warning transmission."
    )

    phase4 = data["phase4_metrics"].copy()
    phase5 = data["warning_transmission"].copy()
    trace = data["network_trace"].copy()

    # ==========================================
    # PHASE 4: AODV EXPERIMENT METRICS
    # ==========================================

    st.subheader("Phase 4 - AODV Network Performance")

    if phase4.empty:

        st.warning("Phase 4 experiment metrics are unavailable.")

    else:

        numeric_cols = [
            "packets_sent",
            "packets_received",
            "pdr_percent",
            "avg_delay_ms",
            "routing_overhead",
            "discovery_time_ms"
        ]

        for col in numeric_cols:
            if col in phase4.columns:
                phase4[col] = pd.to_numeric(
                    phase4[col], errors="coerce"
                )

        total_sent = phase4["packets_sent"].sum()
        total_received = phase4["packets_received"].sum()

        overall_pdr = (
            total_received / total_sent * 100
            if total_sent > 0 else None
        )

        avg_delay = phase4["avg_delay_ms"].mean()
        avg_overhead = phase4["routing_overhead"].mean()
        avg_discovery = phase4["discovery_time_ms"].mean()

        # ----------------------------------
        # PHASE 4 KPI CARDS
        # ----------------------------------

        c1, c2, c3, c4 = st.columns(4)

        c1.metric(
            "Packets Sent",
            f"{total_sent:,.0f}"
        )

        c2.metric(
            "Packets Received",
            f"{total_received:,.0f}"
        )

        c3.metric(
            "Overall Packet Delivery Ratio",
            f"{overall_pdr:.2f}%"
            if overall_pdr is not None else "N/A"
        )

        c4.metric(
            "Mean Experiment Delay",
            f"{avg_delay:.3f} ms"
            if pd.notna(avg_delay) else "N/A"
        )

        c1, c2 = st.columns(2)

        c1.metric(
            "Mean Routing Overhead",
            f"{avg_overhead:.2f}"
            if pd.notna(avg_overhead) else "N/A"
        )

        c2.metric(
            "Mean Route Discovery Time",
            f"{avg_discovery:.3f} ms"
            if pd.notna(avg_discovery) else "N/A"
        )

        st.caption(
            "Overall PDR is calculated from total packets received "
            "divided by total packets sent. Delay, overhead and "
            "discovery time are arithmetic means across experiments."
        )

        st.markdown("---")

        # ----------------------------------
        # EXPERIMENT COMPARISON
        # ----------------------------------

        st.subheader("Experiment-wise Network Metrics")

        metric_fig = px.bar(
            phase4,
            x="experiment",
            y="pdr_percent",
            color="experiment",
            text="pdr_percent",
            title="Packet Delivery Ratio by Experiment",
            labels={
                "experiment": "Experiment",
                "pdr_percent": "PDR (%)"
            },
            template="plotly_dark"
        )

        metric_fig.update_layout(
            yaxis_range=[0, 105],
            showlegend=False
        )

        st.plotly_chart(
            metric_fig,
            use_container_width=True
        )

        c1, c2 = st.columns(2)

        with c1:

            delay_fig = px.bar(
                phase4,
                x="experiment",
                y="avg_delay_ms",
                color="experiment",
                title="Average End-to-End Delay",
                labels={
                    "experiment": "Experiment",
                    "avg_delay_ms": "Delay (ms)"
                },
                template="plotly_dark"
            )

            delay_fig.update_layout(showlegend=False)

            st.plotly_chart(
                delay_fig,
                use_container_width=True
            )

        with c2:

            overhead_fig = px.bar(
                phase4,
                x="experiment",
                y="routing_overhead",
                color="experiment",
                title="Routing Overhead",
                labels={
                    "experiment": "Experiment",
                    "routing_overhead": "Routing Overhead"
                },
                template="plotly_dark"
            )

            overhead_fig.update_layout(showlegend=False)

            st.plotly_chart(
                overhead_fig,
                use_container_width=True
            )

        with st.expander("View Phase 4 Experiment Results"):

            st.dataframe(
                phase4,
                use_container_width=True,
                hide_index=True
            )

            st.download_button(
                "Download Phase 4 Metrics",
                data=phase4.to_csv(index=False),
                file_name="phase4_network_metrics.csv",
                mime="text/csv"
            )

    # ==========================================
    # PHASE 5: WARNING TRANSMISSION
    # ==========================================

    st.markdown("---")

    st.subheader("Phase 5 - Collision Warning Transmission")

    if phase5.empty:

        st.warning("Warning transmission records are unavailable.")

    else:

        if "e2e_delay_ms" in phase5.columns:
            phase5["e2e_delay_ms"] = pd.to_numeric(
                phase5["e2e_delay_ms"], errors="coerce"
            )

        if "hop_count" in phase5.columns:
            phase5["hop_count"] = pd.to_numeric(
                phase5["hop_count"], errors="coerce"
            )

        status = (
            phase5["delivery_status"]
            .astype(str)
            .str.upper()
        )

        delivered = int((status == "DELIVERED").sum())
        total_records = len(phase5)

        delivery_ratio = (
            delivered / total_records * 100
            if total_records > 0 else 0
        )

        mean_warning_delay = phase5[
            "e2e_delay_ms"
        ].mean()

        mean_hops = phase5["hop_count"].mean()

        # ----------------------------------
        # PHASE 5 KPI CARDS
        # ----------------------------------

        c1, c2, c3, c4 = st.columns(4)

        c1.metric(
            "Transmission Records",
            total_records
        )

        c2.metric(
            "Reported Delivered",
            delivered
        )

        c3.metric(
            "Delivery Ratio in Log",
            f"{delivery_ratio:.1f}%"
        )

        c4.metric(
            "Mean Warning Delay",
            f"{mean_warning_delay:.3f} ms"
            if pd.notna(mean_warning_delay) else "N/A"
        )

        st.markdown("---")

        # ----------------------------------
        # DELIVERY STATUS
        # ----------------------------------

        st.subheader("Warning Delivery Status")

        status_counts = (
            phase5["delivery_status"]
            .astype(str)
            .value_counts()
            .reset_index()
        )

        status_counts.columns = [
            "Delivery Status",
            "Records"
        ]

        status_fig = px.pie(
            status_counts,
            names="Delivery Status",
            values="Records",
            hole=0.55,
            title="Warning Transmission Status",
            template="plotly_dark"
        )

        st.plotly_chart(
            status_fig,
            use_container_width=True
        )

        # ----------------------------------
        # WARNING DELAY
        # ----------------------------------

        st.subheader("End-to-End Warning Delay")

        delay_data = phase5.dropna(
            subset=["e2e_delay_ms"]
        )

        if not delay_data.empty:

            delay_warning_fig = px.bar(
                delay_data,
                x="event_id",
                y="e2e_delay_ms",
                color="delivery_status",
                title="Delay for Each Warning Message",
                labels={
                    "event_id": "Warning Event",
                    "e2e_delay_ms": "End-to-End Delay (ms)",
                    "delivery_status": "Delivery Status"
                },
                template="plotly_dark"
            )

            delay_warning_fig.update_layout(
                xaxis_tickangle=-30
            )

            st.plotly_chart(
                delay_warning_fig,
                use_container_width=True
            )

        # ----------------------------------
        # HOP COUNT
        # ----------------------------------

        st.subheader("Routing Hop Count")

        if "hop_count" in phase5.columns:

            hop_counts = (
                phase5["hop_count"]
                .value_counts()
                .sort_index()
                .reset_index()
            )

            hop_counts.columns = [
                "Hop Count",
                "Messages"
            ]

            hop_fig = px.bar(
                hop_counts,
                x="Hop Count",
                y="Messages",
                title="Warning Messages by Hop Count",
                template="plotly_dark"
            )

            st.plotly_chart(
                hop_fig,
                use_container_width=True
            )

            st.info(
                "Use the hop-count distribution to distinguish "
                "single-hop delivery from multi-hop forwarding. "
                "A one-hop record does not demonstrate multi-hop AODV."
            )

        # ----------------------------------
        # WARNING TRANSMISSION TABLE
        # ----------------------------------

        st.subheader("Warning Transmission Records")

        st.dataframe(
            phase5,
            use_container_width=True,
            hide_index=True
        )

        st.download_button(
            "Download Warning Transmission Log",
            data=phase5.to_csv(index=False),
            file_name="warning_transmission_records.csv",
            mime="text/csv"
        )

    # ==========================================
    # PHASE 4: NETWORK TRACE
    # ==========================================

    st.markdown("---")

    st.subheader("Phase 4 - Network Trace")

    if trace.empty:

        st.warning("Network trace data is unavailable.")

    else:

        st.write(
            f"Available network trace records: {len(trace)}"
        )

        st.dataframe(
            trace,
            use_container_width=True,
            hide_index=True
        )

        st.download_button(
            "Download Network Trace",
            data=trace.to_csv(index=False),
            file_name="aodv_network_trace.csv",
            mime="text/csv"
        )


# ==================================================
# PAGE 6: RESULTS GALLERY
# ==================================================


elif page == "Results Gallery":
    st.title("Results Gallery")
    st.markdown(
        "Visual results generated from the SUMO–OMNeT++ simulation and "
        "Phase 5 collision-detection experiments."
    )

    images = get_result_images()

    if not images:
        st.warning("No result images were found in the Phase 5 results folder.")
    else:
        st.success(f"{len(images)} result graphs found.")

        image_descriptions = {
            "phase5_different_lane_ttc_dynamics.png":
                "Different Lane — TTC behaviour when vehicles are in separate lanes.",
            "phase5_high_speed_approach_ttc_dynamics.png":
                "High-Speed Approach — TTC changes when a vehicle approaches another at high speed.",
            "phase5_normal_traffic_ttc_dynamics.png":
                "Normal Traffic — TTC behaviour under ordinary traffic conditions.",
            "phase5_risk_recovery_ttc_dynamics.png":
                "Risk Recovery — How collision risk changes as vehicles move apart or slow down.",
            "phase5_safe_following_ttc_dynamics.png":
                "Safe Following — TTC behaviour when vehicles maintain a safe distance.",
            "phase5_sudden_braking_ttc_dynamics.png":
                "Sudden Braking — TTC behaviour when a leading vehicle brakes suddenly."
        }

        for i in range(0, len(images), 2):
            cols = st.columns(2)

            for j, col in enumerate(cols):
                image_index = i + j

                if image_index < len(images):
                    image_path = images[image_index]
                    image_name = image_path.name
                    description = image_descriptions.get(
                        image_name,
                        "Simulation result generated by the project."
                    )

                    with col:
                        chart_title = (
    image_name
    .replace(".png", "")
    .replace("phase5_", "")
    .replace("_", " ")
    .title()
)

                        st.subheader(chart_title)
                        st.image(str(image_path), use_container_width=True)
                        st.caption(description)

    st.divider()
    st.subheader("Available Result Files")

    result_files = [
        "collision_detection_log.csv",
        "ground_truth_evaluations.csv",
        "phase5_scenario_evaluation_metrics.csv",
        "vehicle_state_log.csv",
        "warning_transmission_log.csv"
    ]

    for filename in result_files:
        file_path = PHASE5_DIR / filename

        if file_path.exists():
            st.write(f"**{filename}** — Available")
        else:
            st.write(f"**{filename}** — Not found")

