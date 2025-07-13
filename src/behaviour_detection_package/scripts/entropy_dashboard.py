import streamlit as st
import pandas as pd
import time
import os
import plotly.express as px
import numpy as np
from datetime import datetime

# Configure page
st.set_page_config(
    page_title="Cognitive Workload Dashboard",
    layout="wide",
    page_icon="🧠"
)

# Custom CSS for styling
st.markdown("""
<style>
    .header {
        color: #1f77b4;
        border-bottom: 2px solid #1f77b4;
        padding-bottom: 10px;
    }
    .metric-card {
        border-radius: 10px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        padding: 20px;
        margin-bottom: 20px;
        text-align: center;
        transition: transform 0.3s;
    }
    .metric-card:hover {
        transform: translateY(-5px);
        box-shadow: 0 6px 12px rgba(0,0,0,0.15);
    }
    .high-load { background-color: #ffcccc; }
    .medium-load { background-color: #ffffcc; }
    .low-load { background-color: #ccffcc; }
    .plot-container {
        background: white;
        border-radius: 10px;
        padding: 20px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        margin-bottom: 20px;
    }
    .status-bar {
        position: fixed;
        bottom: 0;
        left: 0;
        right: 0;
        background: #f0f2f6;
        padding: 10px;
        border-top: 1px solid #ddd;
        z-index: 100;
    }
</style>
""", unsafe_allow_html=True)

# Page header
st.title("🧠 Real-Time Cognitive Workload Monitor")
st.markdown("""
Tracking operator cognitive load through entropy metrics - Updated: *%s*
""" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

# File path - more reliable method
home_dir = os.path.expanduser("~")
log_file = os.path.join(home_dir, "entropy_log.csv")

# Debug info
st.sidebar.header("Configuration")
st.sidebar.info(f"Monitoring file: `{log_file}`")
st.sidebar.info(f"Last checked: {datetime.now().strftime('%H:%M:%S')}")

# Initialize session state
if 'df' not in st.session_state:
    st.session_state.df = pd.DataFrame()
if 'last_update' not in st.session_state:
    st.session_state.last_update = 0
if 'file_exists' not in st.session_state:
    st.session_state.file_exists = os.path.exists(log_file)
if 'error_count' not in st.session_state:
    st.session_state.error_count = 0

# Function to classify cognitive load
def classify_load(entropy_value):
    if entropy_value > 0.7:
        return "High", "high-load", "🔥 High cognitive load detected"
    elif entropy_value > 0.4:
        return "Medium", "medium-load", "⚠️ Moderate cognitive load"
    else:
        return "Low", "low-load", "✅ Normal cognitive load"

# Main dashboard
if not st.session_state.file_exists:
    st.error("🚨 Entropy log file not found. Please ensure the ROS node is running and writing data.")
    
    # Troubleshooting guide
    with st.expander("Troubleshooting Guide"):
        st.markdown("""
        ### Common Solutions:
        1. **Check ROS Node Status**:
           - Verify entropy node is running: `rosnode list | grep entropy`
           - Restart node if needed
        2. **File Permissions**:
           ```bash
           touch ~/entropy_log.csv
           chmod 666 ~/entropy_log.csv
           ```
        3. **Verify Data Path**:
           - The entropy node should write to: `os.path.expanduser("~/entropy_log.csv")`
        4. **Test File Creation**:
           ```python
           import pandas as pd
           pd.DataFrame({
               'timestamp': [time.time()],
               'entropy_ang': [0.5],
               'entropy_lin': [0.3],
               'total_entropy': [0.4]
           }).to_csv("~/entropy_log.csv", index=False)
           ```
        """)
    
    if st.button("Recheck File Existence"):
        st.session_state.file_exists = os.path.exists(log_file)
        st.experimental_rerun()
else:
    try:
        # Read and process data
        current_time = time.time()
        if current_time - st.session_state.last_update > 2.5:  # Update every 2.5 seconds
            try:
                df = pd.read_csv(log_file)
                
                # Validate data structure
                required_cols = ['timestamp', 'entropy_ang', 'entropy_lin', 'total_entropy']
                missing_cols = [col for col in required_cols if col not in df.columns]
                
                if missing_cols:
                    st.error(f"Missing columns in data: {', '.join(missing_cols)}")
                    st.session_state.error_count += 1
                else:
                    # Convert timestamp
                    try:
                        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
                    except:
                        df['timestamp'] = pd.to_datetime(df['timestamp'])
                    
                    df['time_elapsed'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
                    
                    # Calculate cognitive load level
                    if not df.empty:
                        latest = df.iloc[-1]
                        load_level, load_class, status_msg = classify_load(latest['total_entropy'])
                    
                    # Store in session state
                    st.session_state.df = df
                    st.session_state.latest = latest
                    st.session_state.load_level = load_level
                    st.session_state.load_class = load_class
                    st.session_state.status_msg = status_msg
                    st.session_state.error_count = 0
            except pd.errors.EmptyDataError:
                st.warning("Data file is empty. Waiting for data...")
            except Exception as e:
                st.error(f"Data processing error: {str(e)}")
                st.session_state.error_count += 1
            
            st.session_state.last_update = current_time
        
        # Get from session state
        df = st.session_state.df
        
        if df.empty:
            st.warning("Waiting for initial data...")
            time.sleep(2)
            st.experimental_rerun()
        
        latest = st.session_state.latest
        load_level = st.session_state.load_level
        load_class = st.session_state.load_class
        status_msg = st.session_state.status_msg
        
        # Dashboard layout
        # Status bar
        st.markdown(f'<div class="status-bar"><strong>Status:</strong> {status_msg} | Last update: {datetime.now().strftime("%H:%M:%S")}</div>', 
                   unsafe_allow_html=True)
        
        # Metrics row
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            st.markdown(f'<div class="metric-card {load_class}">'
                        f'<h3>Current Load</h3>'
                        f'<h1>{load_level}</h1>'
                        f'<p>{latest["total_entropy"]:.3f}</p>'
                        '</div>', unsafe_allow_html=True)
        
        with col2:
            st.markdown(f'<div class="metric-card">'
                        f'<h3>Angular Entropy</h3>'
                        f'<h1>{latest["entropy_ang"]:.3f}</h1>'
                        f'<p>Last value</p>'
                        '</div>', unsafe_allow_html=True)
        
        with col3:
            st.markdown(f'<div class="metric-card">'
                        f'<h3>Linear Entropy</h3>'
                        f'<h1>{latest["entropy_lin"]:.3f}</h1>'
                        f'<p>Last value</p>'
                        '</div>', unsafe_allow_html=True)
        
        with col4:
            st.markdown(f'<div class="metric-card">'
                        f'<h3>Data Points</h3>'
                        f'<h1>{len(df)}</h1>'
                        f'<p>Since start</p>'
                        '</div>', unsafe_allow_html=True)
        
        # Charts row
        st.markdown("---")
        st.subheader("Cognitive Workload Trends")
        
        col_chart1, col_chart2 = st.columns([2, 1])
        
        with col_chart1:
            st.markdown('<div class="plot-container">', unsafe_allow_html=True)
            fig1 = px.line(
                df, 
                x='timestamp', 
                y=['entropy_ang', 'entropy_lin', 'total_entropy'],
                labels={'value': 'Entropy Value', 'timestamp': 'Time'},
                title='Entropy Components Over Time',
                line_shape='spline'
            )
            fig1.update_layout(
                hovermode='x unified',
                legend_title='Metric',
                height=400,
                xaxis_title='Time',
                yaxis_title='Entropy Value'
            )
            # Add cognitive load thresholds
            fig1.add_hline(y=0.4, line_dash="dash", line_color="orange", 
                           annotation_text="Medium Threshold", 
                           annotation_position="bottom right")
            fig1.add_hline(y=0.7, line_dash="dash", line_color="red", 
                           annotation_text="High Threshold", 
                           annotation_position="top right")
            st.plotly_chart(fig1, use_container_width=True)
            st.markdown('</div>', unsafe_allow_html=True)
        
        with col_chart2:
            st.markdown('<div class="plot-container">', unsafe_allow_html=True)
            # Load level distribution
            if not df.empty:
                load_counts = df['total_entropy'].apply(
                    lambda x: classify_load(x)[0]
                ).value_counts().reset_index()
                load_counts.columns = ['Load Level', 'Count']
                
                fig2 = px.pie(
                    load_counts,
                    names='Load Level',
                    values='Count',
                    title='Load Level Distribution',
                    color='Load Level',
                    color_discrete_map={
                        'High': '#ff6b6b',
                        'Medium': '#ffd166',
                        'Low': '#06d6a0'
                    },
                    hole=0.3
                )
                fig2.update_traces(textposition='inside', textinfo='percent+label')
                st.plotly_chart(fig2, use_container_width=True)
            else:
                st.warning("No data available for pie chart")
            st.markdown('</div>', unsafe_allow_html=True)
        
        # Recent data table
        st.subheader("Recent Entropy Measurements")
        if not df.empty:
            st.dataframe(
                df.tail(10)[['timestamp', 'entropy_ang', 'entropy_lin', 'total_entropy']]
                .style.format({
                    'entropy_ang': '{:.3f}',
                    'entropy_lin': '{:.3f}',
                    'total_entropy': '{:.3f}'
                })
                .applymap(lambda x: 'color: red; font-weight: bold' if x > 0.7 
                          else 'color: orange; font-weight: bold' if x > 0.4 
                          else 'color: green', 
                          subset=['total_entropy'])
                .set_properties(**{'background-color': '#f8f9fa'}),
                height=300
            )
        else:
            st.warning("No data available for display")
        
        # Raw data download
        st.download_button(
            label="📥 Download Full Dataset",
            data=df.to_csv(index=False).encode('utf-8'),
            file_name=f'cognitive_workload_{datetime.now().strftime("%Y%m%d_%H%M")}.csv',
            mime='text/csv'
        )
        
        # System diagnostics
        with st.expander("System Diagnostics"):
            st.write(f"**Last update:** {datetime.fromtimestamp(st.session_state.last_update)}")
            st.write(f"**Data points:** {len(df)}")
            st.write(f"**File size:** {os.path.getsize(log_file)/1024:.2f} KB")
            st.write(f"**Error count:** {st.session_state.error_count}")
            st.code(f"First timestamp: {df['timestamp'].min() if not df.empty else 'N/A'}")
            st.code(f"Last timestamp: {df['timestamp'].max() if not df.empty else 'N/A'}")
    
    except Exception as e:
        st.error(f"Critical error: {str(e)}")
        st.session_state.error_count += 1
        
        if st.session_state.error_count > 5:
            st.error("Too many errors. Dashboard paused.")
            st.stop()

# Add refresh mechanism
if st.sidebar.button('Force Refresh'):
    st.experimental_rerun()

# Auto-refresh every 5 seconds
refresh_rate = st.sidebar.slider("Refresh rate (seconds)", 1, 10, 5)
st.sidebar.write(f"Next auto-refresh in {refresh_rate} seconds")

time.sleep(refresh_rate)
st.experimental_rerun()
