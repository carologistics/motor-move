#!/usr/bin/env python3
# Copyright (c) 2026 Carologistics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(filepath: str) -> pd.DataFrame:
    """Laedt die CSV-Datei und gibt ein DataFrame zurueck."""
    df = pd.read_csv(filepath)
    df["time_rel"] = df["timestamp"] - df["timestamp"].iloc[0]
    return df


def create_pid_plot(df: pd.DataFrame, output_path: str):
    """Erstellt einen 4-teiligen Plot fuer PID-Analyse."""

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.suptitle("PID Tuning Analyse", fontsize=14, fontweight="bold")

    time = df["time_rel"]

    has_goal_err = "goal_err_x" in df.columns
    has_ff = "ff_vel_x" in df.columns and "pid_vel_x" in df.columns

    # --- Subplot 1: Goal Error (Abstand zum Ziel) ---
    ax1 = axes[0]
    if has_goal_err:
        ax1.plot(time, df["goal_err_x"], "r-", label="goal_err_x [m]", linewidth=1.5)
        ax1.plot(time, df["goal_err_y"], "g-", label="goal_err_y [m]", linewidth=1.5)
        ax1.plot(time, df["goal_err_yaw"], "b-", label="goal_err_yaw [rad]", linewidth=1.5)
    else:
        ax1.plot(time, df["error_x"], "r-", label="error_x [m]", linewidth=1.5)
        ax1.plot(time, df["error_y"], "g-", label="error_y [m]", linewidth=1.5)
        ax1.plot(time, df["error_yaw"], "b-", label="error_yaw [rad]", linewidth=1.5)
    ax1.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax1.set_ylabel("Error")
    ax1.set_title("Goal Error (Abstand zum Ziel)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # --- Subplot 2: Tracking Error (Abweichung von Trajektorie) ---
    ax2 = axes[1]
    ax2.plot(time, df["error_x"], "r-", label="track_err_x [m]", linewidth=1.5)
    ax2.plot(time, df["error_y"], "g-", label="track_err_y [m]", linewidth=1.5)
    ax2.plot(time, df["error_yaw"], "b-", label="track_err_yaw [rad]", linewidth=1.5)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax2.set_ylabel("Error")
    ax2.set_title("Tracking Error (Abweichung von Trajektorie-Sollposition)")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # --- Subplot 3: Feedforward vs PID Geschwindigkeit ---
    ax3 = axes[2]
    if has_ff:
        ax3.plot(time, df["ff_vel_x"], "r--", label="ff_vel_x", linewidth=1.2, alpha=0.8)
        ax3.plot(time, df["ff_vel_y"], "g--", label="ff_vel_y", linewidth=1.2, alpha=0.8)
        ax3.plot(time, df["ff_vel_yaw"], "b--", label="ff_vel_yaw", linewidth=1.2, alpha=0.8)
        ax3.plot(time, df["pid_vel_x"], "r-", label="pid_vel_x", linewidth=1.5)
        ax3.plot(time, df["pid_vel_y"], "g-", label="pid_vel_y", linewidth=1.5)
        ax3.plot(time, df["pid_vel_yaw"], "b-", label="pid_vel_yaw", linewidth=1.5)
        ax3.set_title("Feedforward (--) vs PID (-) Geschwindigkeit")
    else:
        ax3.plot(time, df["cmd_vel_x"], "r-", label="cmd_vel_x", linewidth=1.5)
        ax3.plot(time, df["cmd_vel_y"], "g-", label="cmd_vel_y", linewidth=1.5)
        ax3.plot(time, df["cmd_vel_yaw"], "b-", label="cmd_vel_yaw", linewidth=1.5)
        ax3.set_title("cmd_vel (kein FF/PID Breakdown verfuegbar)")
    ax3.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax3.set_ylabel("Velocity [m/s, rad/s]")
    ax3.legend(loc="upper right", fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.3)

    # --- Subplot 4: cmd_vel Total ---
    ax4 = axes[3]
    ax4.plot(time, df["cmd_vel_x"], "r-", label="cmd_vel.x [m/s]", linewidth=1.5)
    ax4.plot(time, df["cmd_vel_y"], "g-", label="cmd_vel.y [m/s]", linewidth=1.5)
    ax4.plot(time, df["cmd_vel_yaw"], "b-", label="cmd_vel.z [rad/s]", linewidth=1.5)
    ax4.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax4.set_xlabel("Zeit [s]")
    ax4.set_ylabel("Velocity Command")
    ax4.set_title("cmd_vel Total (nach Clamping)")
    ax4.legend(loc="upper right")
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"Plot gespeichert: {output_path}")


def print_statistics(df: pd.DataFrame):
    """Gibt Statistiken zur PID-Regelung aus."""
    print("\n" + "=" * 50)
    print("PID Tuning Statistiken")
    print("=" * 50)

    duration = df["time_rel"].iloc[-1]
    print(f"Dauer: {duration:.2f} s")
    print(f"Datenpunkte: {len(df)}")
    print(f"Durchschn. Loop-Rate: {len(df) / duration:.1f} Hz")

    has_goal_err = "goal_err_x" in df.columns

    if has_goal_err:
        print("\nFinaler Goal Error:")
        print(f"  goal_err_x:   {df['goal_err_x'].iloc[-1]:+.4f} m")
        print(f"  goal_err_y:   {df['goal_err_y'].iloc[-1]:+.4f} m")
        print(
            f"  goal_err_yaw: {df['goal_err_yaw'].iloc[-1]:+.4f} rad ({np.degrees(df['goal_err_yaw'].iloc[-1]):+.2f})"
        )

    print("\nFinaler Tracking Error:")
    print(f"  error_x:   {df['error_x'].iloc[-1]:+.4f} m")
    print(f"  error_y:   {df['error_y'].iloc[-1]:+.4f} m")
    print(f"  error_yaw: {df['error_yaw'].iloc[-1]:+.4f} rad ({np.degrees(df['error_yaw'].iloc[-1]):+.2f})")

    print("\nMax. Tracking Error:")
    print(f"  |error_x|:   {df['error_x'].abs().max():.4f} m")
    print(f"  |error_y|:   {df['error_y'].abs().max():.4f} m")
    print(f"  |error_yaw|: {df['error_yaw'].abs().max():.4f} rad ({np.degrees(df['error_yaw'].abs().max()):.2f})")

    print("\nMax. cmd_vel:")
    print(f"  |cmd_vel_x|:   {df['cmd_vel_x'].abs().max():.3f} m/s")
    print(f"  |cmd_vel_y|:   {df['cmd_vel_y'].abs().max():.3f} m/s")
    print(f"  |cmd_vel_yaw|: {df['cmd_vel_yaw'].abs().max():.3f} rad/s")
    print("=" * 50 + "\n")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    csv_path = Path(sys.argv[1]).expanduser().resolve()

    if not csv_path.exists():
        print(f"Fehler: Datei nicht gefunden: {csv_path}")
        sys.exit(1)

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
