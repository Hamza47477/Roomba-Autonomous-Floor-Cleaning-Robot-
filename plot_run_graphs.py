import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import sys

# ----------------------------------------------------
# Parse command-line arguments
# ----------------------------------------------------
parser = argparse.ArgumentParser(description='Generate plots from robot simulation data')
parser.add_argument('--timestamp', type=str, help='Timestamp of the run (e.g., 1772394542754)')
args = parser.parse_args()

if not args.timestamp:
    print("Error: --timestamp argument is required")
    sys.exit(1)

timestamp = args.timestamp

# ----------------------------------------------------
# 1. Load the Data
# ----------------------------------------------------
df_trajectory = pd.read_csv(f'run_{timestamp}_trajectory.csv')
df_dist = pd.read_csv(f'run_{timestamp}_distance_time.csv')
df_gps = pd.read_csv(f'run_{timestamp}_gps_scatter.csv')

# Calculate behaviour stats from trajectory data
behav_counts = df_trajectory['behaviour'].value_counts()
total_samples = len(df_trajectory)
df_behav = pd.DataFrame({
    'behaviour': behav_counts.index,
    'percentage': (behav_counts.values / total_samples * 100).round(1)
})

# Custom parser for events.csv to handle unquoted commas in the 'detail' column
events_data = []
with open(f'run_{timestamp}_events.csv', 'r') as f:
    header = f.readline().strip().split(',')
    for line in f:
        parts = line.strip().split(',')
        if len(parts) >= 4:
            # Re-join details if they contained a comma (e.g., pos=(1.78,-0.78))
            time_sec, tick, event_type = parts[:3]
            detail = ','.join(parts[3:])
            events_data.append([float(time_sec), int(tick), event_type, detail])
            
df_events = pd.DataFrame(events_data, columns=['time_sec', 'tick', 'event_type', 'detail'])

# Set global visual aesthetic (clean background, muted colors)
sns.set_theme(style="whitegrid")

# ----------------------------------------------------
# 2. Behaviour Stats Plot (Bar Chart)
# ----------------------------------------------------
plt.figure(figsize=(8, 6))
ax = sns.barplot(
    data=df_behav.sort_values('percentage', ascending=False), 
    x='behaviour', 
    y='percentage', 
    hue='behaviour',
    palette='viridis',
    legend=False
)
plt.title('Time Spent in Different Behaviours (%)', fontsize=16, fontweight='bold')
plt.xlabel('Behaviour', fontsize=12)
plt.ylabel('Percentage (%)', fontsize=12)
plt.ylim(0, 100)

# Add percentage labels on top of bars
for p in ax.patches:
    ax.annotate(f"{p.get_height():.1f}%", 
                (p.get_x() + p.get_width() / 2., p.get_height()),
                ha='center', va='center', fontsize=11, color='black', 
                xytext=(0, 8), textcoords='offset points')

plt.tight_layout()
plt.savefig(f'plots_{timestamp}_behaviour_stats.png', dpi=300)
plt.close()

# ----------------------------------------------------
# 3. Distance and Battery over Time (Dual Axis Line Chart)
# ----------------------------------------------------
fig, ax1 = plt.subplots(figsize=(10, 6))

color1 = 'tab:blue'
ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Distance (m)', color=color1, fontsize=12)
ax1.plot(df_dist['time_sec'], df_dist['distance_m'], color=color1, linewidth=2.5, label='Distance')
ax1.tick_params(axis='y', labelcolor=color1)

# Create a second y-axis that shares the same x-axis
ax2 = ax1.twinx()  
color2 = 'tab:red'
ax2.set_ylabel('Battery (%)', color=color2, fontsize=12)  
ax2.plot(df_dist['time_sec'], df_dist['battery_pct'], color=color2, linewidth=2.5, linestyle='--', label='Battery')
ax2.tick_params(axis='y', labelcolor=color2)

fig.suptitle('Robot Distance Travelled & Battery Drain', fontsize=16, fontweight='bold')
fig.tight_layout()
plt.savefig(f'plots_{timestamp}_distance_battery.png', dpi=300)
plt.close()

# ----------------------------------------------------
# 4. GPS Trajectory Map (Scatter & Line Mapping)
# ----------------------------------------------------
plt.figure(figsize=(10, 8))
behaviors = df_gps['behaviour'].unique()
palette = sns.color_palette("Set2", len(behaviors)) # distinct colors for paths

# Faint line to show the continuous path sequence
plt.plot(df_gps['gps_x'], df_gps['gps_y'], color='gray', alpha=0.3, linewidth=1.5, zorder=1) 

# Scatter plot for specific coordinate readings & behavior mapping
sns.scatterplot(
    data=df_gps, x='gps_x', y='gps_y', 
    hue='behaviour', palette=palette, 
    s=70, edgecolor='white', alpha=0.9, zorder=2
)

plt.title('Robot GPS Trajectory Map', fontsize=16, fontweight='bold')
plt.xlabel('GPS X Coordinate (m)', fontsize=12)
plt.ylabel('GPS Y Coordinate (m)', fontsize=12)
plt.legend(title='Behaviour State', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()
plt.savefig(f'plots_{timestamp}_gps_trajectory.png', dpi=300)
plt.close()

# ----------------------------------------------------
# 5. Event Frequency Plot (Horizontal Bar Chart)
# ----------------------------------------------------
plt.figure(figsize=(10, 6))
event_counts = df_events['event_type'].value_counts().reset_index()
event_counts.columns = ['event_type', 'count']

sns.barplot(data=event_counts, y='event_type', x='count', hue='event_type', palette='magma', legend=False)
plt.title('Frequency of System Events', fontsize=16, fontweight='bold')
plt.xlabel('Number of Occurrences', fontsize=12)
plt.ylabel('Event Type', fontsize=12)
plt.tight_layout()
plt.savefig(f'plots_{timestamp}_event_frequency.png', dpi=300)
plt.close()

print(f"All visual representations have been rendered and saved successfully.")
print(f"Generated plots:")
print(f"  - plots_{timestamp}_behaviour_stats.png")
print(f"  - plots_{timestamp}_distance_battery.png")
print(f"  - plots_{timestamp}_gps_trajectory.png")
print(f"  - plots_{timestamp}_event_frequency.png")