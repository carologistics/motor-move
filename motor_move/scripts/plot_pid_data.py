#!/usr/bin/env python3
"""
PID Tuning Data Plotter
-----------------------
Liest CSV-Daten aus motor_move und erstellt Analyse-Plots.

Verwendung:
    python3 plot_pid_data.py <csv_file> [output_png]

Beispiel:
    python3 plot_pid_data.py ~/ros2/pid_tuning/20260201_143052/pid_data.csv
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def load_csv(filepath: str) -> pd.DataFrame:
    """Lädt die CSV-Datei und gibt ein DataFrame zurück."""
    df = pd.read_csv(filepath)
    # Zeitstempel relativ zum Start (in Sekunden)
    df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
    return df


def create_pid_plot(df: pd.DataFrame, output_path: str):
    """Erstellt einen 3-teiligen Plot für PID-Analyse."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('PID Tuning Analyse', fontsize=14, fontweight='bold')

    time = df['time_rel']

    # --- Subplot 1: Errors ---
    ax1 = axes[0]
    ax1.plot(time, df['error_x'], 'r-', label='error_x [m]', linewidth=1.5)
    ax1.plot(time, df['error_y'], 'g-', label='error_y [m]', linewidth=1.5)
    ax1.plot(time, df['error_yaw'], 'b-', label='error_yaw [rad]', linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('Error')
    ax1.set_title('Regelfehler (error = target - current)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # --- Subplot 2: cmd_vel (PID Output) ---
    ax2 = axes[1]
    ax2.plot(time, df['cmd_vel_x'], 'r-', label='cmd_vel.x [m/s]', linewidth=1.5)
    ax2.plot(time, df['cmd_vel_y'], 'g-', label='cmd_vel.y [m/s]', linewidth=1.5)
    ax2.plot(time, df['cmd_vel_yaw'], 'b-', label='cmd_vel.z [rad/s]', linewidth=1.5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Velocity Command')
    ax2.set_title('PID-Ausgabe (cmd_vel)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # --- Subplot 3: Target vs Current Position ---
    ax3 = axes[2]
    # Berechne aktuelle Position aus Target - Error
    if 'target_x' in df.columns:
        current_x = df['target_x'] - df['error_x']
        current_y = df['target_y'] - df['error_y']

        ax3.plot(time, df['target_x'], 'r--', label='target_x', linewidth=1, alpha=0.7)
        ax3.plot(time, df['target_y'], 'g--', label='target_y', linewidth=1, alpha=0.7)
        ax3.plot(time, current_x, 'r-', label='current_x', linewidth=1.5)
        ax3.plot(time, current_y, 'g-', label='current_y', linewidth=1.5)

    ax3.set_xlabel('Zeit [s]')
    ax3.set_ylabel('Position [m]')
    ax3.set_title('Zielposition vs. aktuelle Position')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Plot gespeichert: {output_path}")


def print_statistics(df: pd.DataFrame):
    """Gibt Statistiken zur PID-Regelung aus."""
    print("\n" + "="*50)
    print("PID Tuning Statistiken")
    print("="*50)

    duration = df['time_rel'].iloc[-1]
    print(f"Dauer: {duration:.2f} s")
    print(f"Datenpunkte: {len(df)}")
    print(f"Durchschn. Loop-Rate: {len(df)/duration:.1f} Hz")

    print("\nFinale Fehler:")
    print(f"  error_x:   {df['error_x'].iloc[-1]:+.4f} m")
    print(f"  error_y:   {df['error_y'].iloc[-1]:+.4f} m")
    print(f"  error_yaw: {df['error_yaw'].iloc[-1]:+.4f} rad ({np.degrees(df['error_yaw'].iloc[-1]):+.2f}°)")

    print("\nMax. Fehler:")
    print(f"  |error_x|:   {df['error_x'].abs().max():.4f} m")
    print(f"  |error_y|:   {df['error_y'].abs().max():.4f} m")
    print(f"  |error_yaw|: {df['error_yaw'].abs().max():.4f} rad ({np.degrees(df['error_yaw'].abs().max()):.2f}°)")

    print("\nMax. cmd_vel:")
    print(f"  |cmd_vel_x|:   {df['cmd_vel_x'].abs().max():.3f} m/s")
    print(f"  |cmd_vel_y|:   {df['cmd_vel_y'].abs().max():.3f} m/s")
    print(f"  |cmd_vel_yaw|: {df['cmd_vel_yaw'].abs().max():.3f} rad/s")
    print("="*50 + "\n")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = Path(sys.argv[1]).expanduser().resolve()

    if not csv_path.exists():
        print(f"Fehler: Datei nicht gefunden: {csv_path}")
        sys.exit(1)

    # Output-Pfad: gleicher Ordner, pid_analysis.png
    if len(sys.argv) >= 3:
        output_path = Path(sys.argv[2]).expanduser().resolve()
    else:
        output_path = csv_path.parent / "pid_analysis.png"

    print(f"Lade CSV: {csv_path}")
    df = load_csv(csv_path)

    print_statistics(df)
    create_pid_plot(df, str(output_path))


if __name__ == "__main__":
    main()
