import streamlit as st
import pandas as pd
import time
import os
import plotly.express as px
import numpy as np

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
</style>
""", unsafe_allow_html=True)

# Page header
st.title("🧠 Real-Time Cognitive Workload Monitor")
st.markdown("""
Tracking operator cognitive load through entropy metrics and physiological indicators
""")

# File path
log_file = os.path.expanduser("~/entropy_log.csv")
placeholder = st.empty()

# Initialize session state
if 'df' not in st.session_state:
    st.session_state.df = pd.DataFrame()
if 'last_update' not in st.session_state:
    st.session_state.last_update = 0

# Function to classify cognitive load
def classify_load(entropy_value):
    if entropy_value > 0.7:
        return "High", "high-load"
    elif entropy_value > 0.4:
        return "Medium", "medium-load"
    else:
        return "Low", "low-load"

# Main dashboard
if not os.path.exists(log_file):
    st.error("🚨 Entropy log file not found. Please ensure the ROS node is running and writing data.")
    st.info("""
    **Troubleshooting tips:**
    1. Verify ROS entropy node is running
    2. Check file permissions for `~/entropy_log.csv`
    3. Ensure ROS node is writing to the correct path
    """)
else:
    while True:
        try:
            # Read and process data
            current_time = time.time()
            if current_time - st.session_state.last_update > 2.5:  # Update every 2.5 seconds
                df = pd.read_csv(log_file)
                
                # Ensure required columns exist
                required_cols = ['timestamp', 'entropy_ang', 'entropy_lin', 'total_entropy']
                if not all(col in df.columns for col in required_cols):
                    st.warning("Waiting for data with required columns...")
                    time.sleep(2)
                    continue
                
                # Convert timestamp
                df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
                df['time_elapsed'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
                
                # Calculate cognitive load level
                if not df.empty:
                    latest = df.iloc[-1]
                    load_level, load_class = classify_load(latest['total_entropy'])
                
                # Store in session state
                st.session_state.df = df
                st.session_state.last_update = current_time
                st.session_state.latest = latest
                st.session_state.load_level = load_level
                st.session_state.load_class = load_class
            
            # Get from session state
            df = st.session_state.df
            latest = st.session_state.latest
            load_level = st.session_state.load_level
            load_class = st.session_state.load_class
            
            # Dashboard layout
            with placeholder.container():
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
                        title='Entropy Components Over Time'
                    )
                    fig1.update_layout(
                        hovermode='x unified',
                        legend_title='Metric',
                        height=400
                    )
                    st.plotly_chart(fig1, use_container_width=True)
                    st.markdown('</div>', unsafe_allow_html=True)
                
                with col_chart2:
                    st.markdown('<div class="plot-container">', unsafe_allow_html=True)
                    # Load level distribution
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
                        }
                    )
                    fig2.update_traces(textposition='inside', textinfo='percent+label')
                    st.plotly_chart(fig2, use_container_width=True)
                    st.markdown('</div>', unsafe_allow_html=True)
                
                # Recent data table
                st.subheader("Recent Entropy Measurements")
                st.dataframe(
                    df.tail(10)[['timestamp', 'entropy_ang', 'entropy_lin', 'total_entropy']]
                    .style.format({
                        'entropy_ang': '{:.3f}',
                        'entropy_lin': '{:.3f}',
                        'total_entropy': '{:.3f}'
                    })
                    .applymap(lambda x: 'color: red' if x > 0.7 else 'color: orange' if x > 0.4 else 'color: green', 
                              subset=['total_entropy'])
                    .set_properties(**{'background-color': '#f8f9fa'}),
                    height=300
                )
                
                # Raw data download
                st.download_button(
                    label="📥 Download Full Dataset",
                    data=df.to_csv(index=False).encode('utf-8'),
                    file_name='cognitive_workload_data.csv',
                    mime='text/csv'
                )
            
            time.sleep(0.5)  # Small sleep to prevent high CPU usage
            
        except pd.errors.EmptyDataError:
            st.warning("Data file is empty. Waiting for data...")
            time.sleep(2)
        except Exception as e:
            st.error(f"Error occurred: {str(e)}")
            st.info("Trying to reconnect in 5 seconds...")
            time.sleep(5)
