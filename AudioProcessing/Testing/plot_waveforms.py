# ==============================================================================
# UMA-8 Waveform Visualization and Analysis Script
# ==============================================================================
#
# Description:
# This script reads multichannel audio data from a CSV file, performs a basic
# analysis for peak amplitude (loudness), and generates two plots:
#   1. A subplot for each individual channel's waveform.
#   2. An overlay plot of all channels for easy comparison of timing.
#
# Dependencies:
# - pandas: For easy CSV file reading.
# - matplotlib: For plotting the graphs.
#
# Installation:
# pip install pandas matplotlib
#
# Usage:
# python plot_waveforms.py
#
# ==============================================================================

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- Configuration ---
CSV_FILENAME = "uma8_capture.csv"
SAMPLE_RATE = 48000 # This must match the sample rate in the C++ program

def analyze_and_plot_waveforms():
    """Reads, analyzes, and plots waveform data from a CSV file."""
    print(f"Reading audio data from '{CSV_FILENAME}'...")
    try:
        df = pd.read_csv(CSV_FILENAME)
    except FileNotFoundError:
        print(f"Error: The file '{CSV_FILENAME}' was not found.")
        print("Please run the C++ capture program first to generate the data file.")
        return
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return

    num_channels = len(df.columns)
    num_samples = len(df)

    # --- 1. LOUDNESS ANALYSIS ---
    print("\n--- Peak Amplitude Analysis (Loudness) ---")
    peak_amplitudes = {}
    for channel_name in df.columns:
        # Find the maximum absolute value in the column
        peak_amplitudes[channel_name] = df[channel_name].abs().max()
    
    # Sort the channels by peak amplitude in descending order
    sorted_peaks = sorted(peak_amplitudes.items(), key=lambda item: item[1], reverse=True)
    
    for i, (channel, peak) in enumerate(sorted_peaks):
        print(f"{i+1}. {channel}: {peak:.4f}")
    
    # --- 2. PLOTTING ---
    # Create a time axis in seconds for the x-axis of the plot
    time_axis = np.arange(num_samples) / SAMPLE_RATE
    
    print("\nGenerating plots... Close the plot windows to exit.")
    
    # --- Plot 1: Individual Subplots ---
    fig1, axes = plt.subplots(num_channels, 1, figsize=(15, 10), sharex=True, sharey=True)
    fig1.suptitle(f'UMA-8 Individual Waveforms ({num_samples} samples)', fontsize=16)
    for i, channel_name in enumerate(df.columns):
        ax = axes[i]
        ax.plot(time_axis, df[channel_name])
        ax.set_ylabel(channel_name.replace('_', ' '), rotation=0, labelpad=40, ha='right')
        ax.grid(True)
    axes[-1].set_xlabel("Time (seconds)")
    fig1.tight_layout(rect=[0, 0.03, 1, 0.96])
    
    # --- Plot 2: Overlay Plot for Timing Comparison ---
    fig2, ax2 = plt.subplots(figsize=(15, 8))
    fig2.suptitle('All Channels Overlay (For Timing Analysis)', fontsize=16)
    for channel_name in df.columns:
        ax2.plot(time_axis, df[channel_name], label=channel_name)
    
    ax2.set_xlabel("Time (seconds)")
    ax2.set_ylabel("Amplitude")
    ax2.grid(True)
    ax2.legend()
    fig2.tight_layout(rect=[0, 0.03, 1, 0.96])

    print("\n--- How to Analyze the Plots ---")
    print("1. The console output above shows which mic detected the loudest sound.")
    print("2. In the 'All Channels Overlay' plot window, use the ZOOM tool (magnifying glass)")
    print("   to click and drag a box around the very beginning of the sound event.")
    print("3. As you zoom in, you will be able to see which colored line rises from zero first.")
    
    plt.show()

if __name__ == "__main__":
    analyze_and_plot_waveforms()