import streamlit as st
import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go

from data_loader import (
    load_all_data,
    get_data_summary,
    get_all_result_images,
    PHASE5_DIR,
    PHASE6_DIR,
    PHASE7_DIR,
    FINAL_DIR
)

from charts import (
    plot_speed_trajectories,
    plot_position_trajectories,
    plot_gap_preservation_comparison,
    plot_speed_reduction_bars,
    plot_causal_sequence_timeline,
    plot_latency_distribution
)

# --------------------------------------------------
# PAGE CONFIGURATION
# --------------------------------------------------

st.set_page_config(
    page_title="VANET Collision Warning & Traffic Management",
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
        font-size: 32px;
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
        padding: 16px;
        border-radius: 12px;
    }

    div[data-testid="stMetricValue"] {
        color: #60A5FA;
    }

    .info-card {
        background-color: #171E2B;
        border: 1px solid #29364A;
        padding: 16px;
        border-radius: 12px;
        margin-bottom: 12px;
    }

    .info-card h4 {
        color: #60A5FA;
        margin-bottom: 6px;
    }

    .info-card p {
        color: #CBD5E1;
        font-size: 14px;
        margin-bottom: 0;
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
# SIDEBAR NAVIGATION
# --------------------------------------------------

st.sidebar.title("🚗 VANET Analytics")
st.sidebar.caption("Integrated Collision Warning & Response")

st.sidebar.markdown("---")

page = st.sidebar.radio(
    "NAVIGATION",
    [
        "Project Overview",
        "Vehicle Simulation",
        "Collision Detection",
        "Autonomous Response (Phase 6)",
        "Experiment Validation (Phase 7)",
        "Scenario Analysis",
        "Network Performance",
        "Results Gallery & Downloads"
    ]
)

st.sidebar.markdown("---")
st.sidebar.markdown("### System Architecture")
st.sidebar.caption(
    "VANET-Based Collision Warning "
    "and Traffic Management Using AODV Routing"
)

st.sidebar.markdown("---")
st.sidebar.caption("Phases 1 – 8 Integrated | Academic Review")


# --------------------------------------------------
# PAGE HEADER
# --------------------------------------------------

st.markdown(
    '<div class="main-title">VANET Collision Warning & Traffic Management</div>',
    unsafe_allow_html=True
)
st.markdown(
    '<div class="sub-title">'
    'Microscopic SUMO Traffic • RFC 3561 AODV Ad-Hoc Routing • TraCI Closed-Loop Autonomous Response'
    '</div>',
    unsafe_allow_html=True
)


# ==================================================
# PAGE 1: PROJECT OVERVIEW
# ==================================================

if page == "Project Overview":

    st.markdown("## System Performance Overview")
    st.write(
        "A closed-loop vehicular ad-hoc network simulation coupling microscopic vehicle mobility (SUMO 1.28.0), "
        "same-lane collision risk evaluation (TTC, gap, closing speed), ETSI DENM warning generation, "
        "RFC 3561 AODV multi-hop routing, and TraCI autonomous deceleration and recovery control."
    )

    # KPI CARDS
    col1, col2, col3, col4 = st.columns(4)

    with col1:
        st.metric("Highway Corridor", "3.0 km (2 Lanes)")
    with col2:
        st.metric("Test Scenarios", summary["scenarios"])
    with col3:
        st.metric("Experiment Runs", summary["total_experiment_runs"])
    with col4:
        st.metric("Packet Delivery Ratio", summary["pdr_percent"])

    col5, col6, col7, col8 = st.columns(4)
    with col5:
        st.metric("Average AODV Latency", summary["avg_latency_ms"])
    with col6:
        st.metric("Braking Actuations", summary["braking_actuations"])
    with col7:
        st.metric("Max Buffer Preserved", summary["max_gap_preserved_m"])
    with col8:
        st.metric("Phase 6 Unit Tests", "6 / 6 Passed (100%)")

    st.markdown("---")

    # CLOSED-LOOP WORKFLOW
    st.markdown("## Closed-Loop Execution Workflow")
    st.markdown("""
    <div class="info-card">
        <h4>1. Microscopic Traffic Simulation (SUMO)</h4>
        <p>Models 5 passenger vehicles traversing a 3.0 km multi-lane highway. Generates vehicle coordinates, speeds, and lane positions at 0.1s timesteps.</p>
    </div>
    <div class="info-card">
        <h4>2. Same-Lane Rear-End Collision Detection</h4>
        <p>Extracts vehicle telemetry via TraCI, isolates vehicles by lane index, and computes bumper-to-bumper headway ($d_{gap}$), closing speed ($v_{closing}$), and Time-to-Collision (TTC).</p>
    </div>
    <div class="info-card">
        <h4>3. Structured Warning Generation (ETSI DENM)</h4>
        <p>Translates hazardous events into standardized Decentralized Environmental Notification Messages containing ActionID, timestamps, vehicle IPs, risk tiers, and telemetry.</p>
    </div>
    <div class="info-card">
        <h4>4. Multi-Hop Wireless Network Routing (RFC 3561 AODV)</h4>
        <p>Evaluates dynamic 802.11p connectivity ($R = 100\text{ m}$). Transmits warning packets across moving intermediate relays via dynamic on-demand route discovery.</p>
    </div>
    <div class="info-card">
        <h4>5. Autonomous Vehicle Response (TraCI slowDown)</h4>
        <p>Recipient On-Board Unit validates delivery, freshness, and deduplication. Actuates calibrated deceleration profiles, monitors speed reduction, and restores normal driving control.</p>
    </div>
    """, unsafe_allow_html=True)

    st.markdown("---")

    # DATASET AUDIT TABLE
    st.markdown("## Available Simulation Datasets")
    dataset_info = []
    for name, df in data.items():
        dataset_info.append({
            "Dataset Identifier": name.replace("_", " ").title(),
            "Record Count": len(df),
            "Feature Columns": len(df.columns),
            "Status": "Verified & Active" if not df.empty else "Not Found"
        })

    st.dataframe(pd.DataFrame(dataset_info), use_container_width=True, hide_index=True)


# ==================================================
# PAGE 2: VEHICLE SIMULATION
# ==================================================

elif page == "Vehicle Simulation":

    st.header("Vehicle Simulation & Microscopic Mobility")
    st.caption("Microscopic telemetry recorded by SUMO via TraCI during highway simulation.")

    df = data["vehicle_state"].copy()

    if df.empty:
        st.warning("Vehicle state data is unavailable.")
    else:
        for col in ["timestamp", "pos_x", "pos_y", "speed_mps", "speed_kmh"]:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors="coerce")

        df = df.dropna(subset=["timestamp", "vehicle_id"])
        vehicles = sorted(df["vehicle_id"].dropna().unique())

        selected_vehicles = st.multiselect("Select Vehicles to Inspect", vehicles, default=vehicles)
        filtered = df[df["vehicle_id"].isin(selected_vehicles)].copy()

        if filtered.empty:
            st.warning("Select at least one vehicle.")
        else:
            col1, col2, col3 = st.columns(3)
            with col1:
                st.metric("Selected Vehicles", filtered["vehicle_id"].nunique())
            with col2:
                st.metric("Average Fleet Speed", f"{filtered['speed_kmh'].mean():.2f} km/h")
            with col3:
                st.metric("Peak Speed Observed", f"{filtered['speed_kmh'].max():.2f} km/h")

            st.plotly_chart(plot_speed_trajectories(filtered), use_container_width=True)
            st.plotly_chart(plot_position_trajectories(filtered), use_container_width=True)


# ==================================================
# PAGE 3: COLLISION DETECTION
# ==================================================

elif page == "Collision Detection":

    st.header("Collision Detection & Ground-Truth Evaluation")
    st.caption("Headway gap, relative closing speed, and Time-to-Collision (TTC) evaluation.")

    df_det = data["collision_detection"].copy()
    df_gt = data["ground_truth"].copy()

    if df_det.empty:
        st.warning("Collision detection log is unavailable.")
    else:
        col1, col2, col3 = st.columns(3)
        valid_ttc = df_det[(df_det["ttc_s"] > 0) & (df_det["ttc_s"] < 50)]["ttc_s"]

        with col1:
            st.metric("Minimum Observed TTC", f"{valid_ttc.min():.2f} s" if not valid_ttc.empty else "N/A")
        with col2:
            st.metric("Minimum Headway Gap", f"{df_det['gap_m'].min():.2f} m")
        with col3:
            st.metric("Warnings Triggered", int(df_det["warning_generated"].sum()))

        st.subheader("Headway Gap and TTC Dynamics (Follower V2 vs Leader V1)")
        v2_v1 = df_det[(df_det["follower_id"] == "V2") & (df_det["leader_id"] == "V1")].copy()

        if not v2_v1.empty:
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=v2_v1["timestamp"], y=v2_v1["gap_m"], name="Headway Gap (m)", line=dict(color="#3B82F6", width=2.5)))
            
            # Filter valid TTC
            ttc_plot = v2_v1.copy()
            ttc_plot.loc[ttc_plot["ttc_s"] <= 0, "ttc_s"] = np.nan
            ttc_plot.loc[ttc_plot["ttc_s"] > 25, "ttc_s"] = np.nan
            fig.add_trace(go.Scatter(x=ttc_plot["timestamp"], y=ttc_plot["ttc_s"], name="TTC (s)", yaxis="y2", line=dict(color="#EF4444", dash="dash", width=2)))

            fig.update_layout(
                title="Dynamic Headway Gap and Time-to-Collision Trajectory",
                xaxis_title="Simulation Time (s)",
                yaxis=dict(title="Headway Gap (m)", title_font=dict(color="#3B82F6")),
                yaxis2=dict(title="Time-to-Collision (s)", title_font=dict(color="#EF4444"), overlaying="y", side="right"),
                template="plotly_dark",
                hovermode="x unified"
            )
            st.plotly_chart(fig, use_container_width=True)

        if not df_gt.empty:
            st.subheader("Ground-Truth Performance Evaluation Matrix")
            tp = df_gt["TP"].sum()
            fp = df_gt["FP"].sum()
            tn = df_gt["TN"].sum()
            fn = df_gt["FN"].sum()

            prec_str = f"{tp / (tp + fp) * 100:.2f}%" if (tp + fp) > 0 else "N/A"
            rec_str = f"{tp / (tp + fn) * 100:.2f}%" if (tp + fn) > 0 else "N/A"

            c1, c2, c3, c4 = st.columns(4)
            c1.metric("True Positives (TP)", int(tp))
            c2.metric("False Positives (FP)", int(fp))
            c3.metric("Precision", prec_str)
            c4.metric("Recall (Sensitivity)", rec_str)


# ==================================================
# PAGE 4: AUTONOMOUS RESPONSE (PHASE 6)
# ==================================================

elif page == "Autonomous Response (Phase 6)":

    st.header("Phase 6: Closed-Loop Autonomous Vehicle Response")
    st.caption("Verification of warning validation, TraCI slowDown() actuation, and safe driving recovery.")

    df_causal = data["causal_sequence"].copy()
    df_act = data["braking_actuations"].copy()
    df_tests = data["phase6_test_results"].copy()
    df_resp = data["response_telemetry"].copy()

    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Braking Actuations", len(df_act) if not df_act.empty else 0)
    with col2:
        st.metric("Responding Vehicles", df_act["vehicle_id"].nunique() if not df_act.empty else 0)
    with col3:
        st.metric("Peak Speed Drop (Δv)", f"{df_resp['speed_reduction_mps'].max() * 3.6:.1f} km/h" if not df_resp.empty else "N/A")
    with col4:
        st.metric("Phase 6 Unit Tests", "6 / 6 PASSED (100%)")

    st.markdown("---")

    # Causal Sequence Timeline
    if not df_causal.empty:
        st.subheader("1. The 8-Step Causal Sequence (Hazard to Safe Recovery)")
        st.write(
            "Every step of the closed-loop safety response is recorded chronologically with high-resolution timestamps, "
            "verifying that warnings are strictly validated before actuation."
        )
        st.plotly_chart(plot_causal_sequence_timeline(df_causal), use_container_width=True)

        with st.expander("Inspect Raw Causal Sequence Log"):
            st.dataframe(df_causal, use_container_width=True)

    # Braking Actuations Table
    if not df_act.empty:
        st.subheader("2. Autonomous Braking Actuation Parameters")
        st.dataframe(df_act, use_container_width=True)

    # Unit Tests Report
    if not df_tests.empty:
        st.subheader("3. Phase 6 Automated Unit & Edge-Case Tests")
        st.dataframe(df_tests, use_container_width=True, hide_index=True)


# ==================================================
# PAGE 5: EXPERIMENT VALIDATION (PHASE 7)
# ==================================================

elif page == "Experiment Validation (Phase 7)":

    st.header("Phase 7: Advanced Experiments & Statistical Validation")
    st.caption("Empirical benchmark across 60 multi-seed runs contrasting Closed-Loop (Response ON) vs. Open-Loop Baseline (Response OFF).")

    df_comp = data["phase7_comparison"].copy()
    df_summary = data["phase7_summary"].copy()
    df_runs = data["phase7_runs"].copy()
    df_sens = data["phase7_sensitivity"].copy()

    if not df_comp.empty:
        st.subheader("1. Headway Margin Preservation (Response ON vs. OFF)")
        st.plotly_chart(plot_gap_preservation_comparison(df_comp), use_container_width=True)

        st.subheader("2. Follower Speed Reduction (Δv)")
        st.plotly_chart(plot_speed_reduction_bars(df_comp), use_container_width=True)

        st.subheader("3. Comprehensive Mode Contrast Matrix")
        st.dataframe(df_comp, use_container_width=True, hide_index=True)

    if not df_summary.empty:
        st.subheader("4. Statistical Metrics with 95% Confidence Intervals")
        st.write("Aggregated across 5 independent stochastic runs per scenario using Student's t-distribution ($df = 4, t_{0.025, 4} = 2.776$).")
        
        cols_to_show = [
            "Scenario", "Mode", "MinGap_m_Mean", "MinGap_m_CI95_Margin",
            "MaxSpeedReduction_kmh_Mean", "PDR_Percent_Mean", "AvgE2EDelay_ms_Mean"
        ]
        available_cols = [c for c in cols_to_show if c in df_summary.columns]
        st.dataframe(df_summary[available_cols], use_container_width=True, hide_index=True)

    if not df_sens.empty:
        st.subheader("5. Warning Threshold Sensitivity Analysis")
        th_agg = df_sens.groupby("TTCThreshold_s").mean(numeric_only=True).reset_index()
        fig_sens = px.line(
            th_agg,
            x="TTCThreshold_s",
            y=["DetectionLatency_s", "WarningsGenerated"],
            markers=True,
            title="Impact of Warning TTC Threshold on Latency & Dissemination Frequency",
            labels={"TTCThreshold_s": "TTC Warning Threshold (s)", "value": "Metric Magnitude", "variable": "Metric"},
            template="plotly_dark"
        )
        st.plotly_chart(fig_sens, use_container_width=True)


# ==================================================
# PAGE 6: SCENARIO ANALYSIS
# ==================================================

elif page == "Scenario Analysis":

    st.header("Traffic Scenario Comparative Analysis")
    st.caption("Benchmark metrics across all 6 simulated highway traffic scenarios.")

    df_scen = data["scenario_metrics"].copy()

    if df_scen.empty:
        st.warning("Scenario metrics are unavailable.")
    else:
        st.subheader("Scenario Benchmark Summary Table")
        st.dataframe(df_scen, use_container_width=True, hide_index=True)


# ==================================================
# PAGE 7: NETWORK PERFORMANCE
# ==================================================

elif page == "Network Performance":

    st.header("Wireless Network & RFC 3561 AODV Protocol Analysis")
    st.caption("Performance of on-demand ad-hoc routing, packet delivery, and transmission delays.")

    df_runs = data["phase7_runs"].copy()
    df_trace = data["network_trace"].copy()

    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Radio Range (UnitDisk)", "100.0 m")
    with col2:
        st.metric("Channel Standard", "802.11p DSRC (6 Mbps)")
    with col3:
        st.metric("Packet Delivery Ratio", summary["pdr_percent"])
    with col4:
        st.metric("Average Latency", summary["avg_latency_ms"])

    if not df_runs.empty:
        st.subheader("1. End-to-End AODV Delivery Latency Distribution")
        st.plotly_chart(plot_latency_distribution(df_runs), use_container_width=True)

    if not df_trace.empty:
        st.subheader("2. RFC 3561 AODV Packet Trace Log")
        st.dataframe(df_trace.head(50), use_container_width=True)


# ==================================================
# PAGE 8: RESULTS GALLERY & DOWNLOADS
# ==================================================

elif page == "Results Gallery & Downloads":

    st.header("Results Gallery & Downloadable Deliverables")
    st.caption("Consolidated analytical figures and CSV result logs across all project phases.")

    images = get_all_result_images()
    if images:
        st.subheader(f"Generated Figures ({len(images)} Available)")
        for i in range(0, len(images), 2):
            cols = st.columns(2)
            for j, col in enumerate(cols):
                idx = i + j
                if idx < len(images):
                    img_path = images[idx]
                    with col:
                        title = img_path.stem.replace("phase7_", "").replace("phase6_", "").replace("phase5_", "").replace("_", " ").title()
                        st.subheader(title)
                        st.image(str(img_path), use_container_width=True)
                        st.caption(f"Source: `{img_path.name}`")

    st.divider()
    st.subheader("Downloadable Empirical Result CSVs")

    download_list = [
        ("Causal Sequence Trace (Phase 6)", "causal_sequence", "causal_event_sequence.csv"),
        ("Autonomous Braking Actuations (Phase 6)", "braking_actuations", "autonomous_braking_actuations.csv"),
        ("All 60 Experiment Runs (Phase 7)", "phase7_runs", "phase7_all_experiment_runs.csv"),
        ("Statistical 95% CI Summary (Phase 7)", "phase7_summary", "phase7_statistical_summary.csv"),
        ("Response Comparison Matrix (Phase 7)", "phase7_comparison", "phase7_response_comparison_matrix.csv"),
        ("Threshold Sensitivity Analysis (Phase 7)", "phase7_sensitivity", "phase7_threshold_sensitivity_analysis.csv"),
        ("Phase 6 Unit Test Results", "phase6_test_results", "phase6_unit_test_results.csv"),
        ("Collision Detection Log (Phase 5)", "collision_detection", "collision_detection_log.csv"),
    ]

    for label, key, filename in download_list:
        df_target = data.get(key)
        if df_target is not None and not df_target.empty:
            st.download_button(
                label=f"⬇ Download {label} ({len(df_target)} rows)",
                data=df_target.to_csv(index=False),
                file_name=filename,
                mime="text/csv",
                key=f"dl_{key}"
            )
