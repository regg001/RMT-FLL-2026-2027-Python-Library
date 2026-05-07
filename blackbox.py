from pybricks.tools import wait, StopWatch

# ============================================================
# blackbox.py — FLL 2026-2027
# Persistent run logger with CSV export for Excel/Sheets.
#
# USAGE IN MISSION FILES:
#   from blackbox import BlackBox
#   box = BlackBox()
#   box.start_run("Mission 1", speed=450)
#   # ... your mission code ...
#   box.save(bot.report_card)
#
# READING LOGS AFTER COMPETITION:
#   - Full text log:  run read_log.py in VSCode (F5)
#   - CSV export:     download blackbox.csv from hub via code.pybricks.com
#                     open in Excel or Google Sheets for visualization
#
# FILES STORED ON HUB:
#   blackbox.txt    — human-readable full log (append mode)
#   blackbox.csv    — structured data for Excel/Sheets (append mode)
#   run_count.txt   — single integer, persists run number across reboots
#   session_id.txt  — increments each time the hub is powered on
# ============================================================


class BlackBox:
    """
    Persistent run logger for FLL competition data analysis.

    Writes two files every run:
      - blackbox.txt  for human-readable terminal review
      - blackbox.csv  for Excel/Google Sheets visualization

    Both files are append-only — data accumulates across the
    entire season until you explicitly call clear().
    """

    TXT_FILE     = "blackbox.txt"
    CSV_FILE     = "blackbox.csv"
    COUNT_FILE   = "run_count.txt"
    SESSION_FILE = "session_id.txt"

    def __init__(self):
        self.run_count  = self._load_int(self.COUNT_FILE,   default=0)
        self.session_id = self._load_int(self.SESSION_FILE, default=0)

        # These are set by start_run() before each mission
        self.current_mission = "Unknown"
        self.current_speed   = 0
        self.timer           = StopWatch()

        # Track moves logged this run for session_summary()
        self._session_runs = []

        # Increment session ID on every new BlackBox() — one per power cycle
        self.session_id += 1
        self._save_int(self.SESSION_FILE, self.session_id)

        # Write CSV header if the file is new
        self._ensure_csv_header()

        print("BlackBox ready. Session: {} | All-time runs: {}".format(
            self.session_id, self.run_count))

    # ────────────────────────────────────────────────────────────────────────
    # PUBLIC API
    # ────────────────────────────────────────────────────────────────────────

    def start_run(self, mission_name, speed):
        """
        Call at the start of each mission, before gyro_reset().

        Args:
            mission_name: descriptive name (e.g. "Mission 1" or "Watercraft")
            speed:        drive speed used this run in mm/s
        """
        self.run_count       += 1
        self.current_mission  = mission_name
        self.current_speed    = speed
        self.timer.reset()

        self._save_int(self.COUNT_FILE, self.run_count)

        print("BlackBox: Run #{} | {} | {}mm/s".format(
            self.run_count, mission_name, speed))

    def save(self, report_card):
        """
        Call at the end of each mission with bot.report_card.

        Writes one entry to both blackbox.txt and blackbox.csv.

        Args:
            report_card: list of (move_name, target, error) tuples
                         from the Robot class diagnostic system
        """
        if not report_card:
            print("BlackBox: Nothing to save — report_card is empty.")
            return

        # Compute summary statistics
        total_err = sum(abs(e) for _, _, e in report_card)
        avg_err   = total_err / len(report_card)
        bias      = sum(e for _, _, e in report_card)
        bias_dir  = "RIGHT" if bias > 0 else "LEFT" if bias < 0 else "NONE"
        elapsed   = self.timer.time()   # ms since start_run()

        # Save to both formats
        self._write_txt(report_card, total_err, avg_err, bias, bias_dir, elapsed)
        self._write_csv(report_card, elapsed)

        # Track for session summary
        self._session_runs.append({
            "run":     self.run_count,
            "mission": self.current_mission,
            "avg_err": avg_err,
            "bias":    bias,
            "moves":   len(report_card),
            "elapsed": elapsed,
        })

        print("BlackBox: Run #{} saved. Avg error: {:.2f}° | Bias: {:.2f}°".format(
            self.run_count, avg_err, bias))

    def print_history(self):
        """
        Print the full text log to the VSCode terminal.
        Run this via read_log.py after competition for full review.
        """
        print("\n" + "=" * 40)
        print("  BLACKBOX FULL HISTORY")
        print("=" * 40)
        try:
            with open(self.TXT_FILE, "r") as f:
                print(f.read())
        except OSError:
            print("No blackbox data found. Run some missions first.")
        print("=" * 40 + "\n")

    def session_summary(self):
        """
        Print a quick summary of this session's runs only.
        Use between runs at competition — no laptop needed.
        """
        if not self._session_runs:
            print("No runs recorded this session.")
            return

        print("\n" + "=" * 40)
        print("  SESSION {} SUMMARY".format(self.session_id))
        print("=" * 40)
        print("{:5} | {:15} | {:8} | {:6}".format(
            "RUN", "MISSION", "AVG ERR", "BIAS"))
        print("-" * 40)

        for r in self._session_runs:
            print("{:5} | {:15} | {:7.2f}° | {:+.2f}°".format(
                r["run"], r["mission"][:15],
                r["avg_err"], r["bias"]))

        # Session statistics
        session_avg = sum(r["avg_err"] for r in self._session_runs) / len(self._session_runs)
        session_bias = sum(r["bias"] for r in self._session_runs) / len(self._session_runs)

        print("=" * 40)
        print("Session avg error: {:.2f}°".format(session_avg))
        print("Session avg bias:  {:+.2f}°".format(session_bias))

        if abs(session_bias) > 2:
            direction = "RIGHT" if session_bias > 0 else "LEFT"
            print("WARNING: Consistent {} bias — check TURN_FLOOR".format(direction))

        print("=" * 40 + "\n")

    def clear(self, confirm=False):
        """
        Permanently delete all log data. Cannot be undone.

        Must pass confirm=True to prevent accidental calls.
        Run count and session ID are also reset.

        Usage:
            box.clear(confirm=True)
        """
        if not confirm:
            print("BlackBox: clear() requires confirm=True.")
            print("Call box.clear(confirm=True) to wipe all data.")
            return

        for filename in [self.TXT_FILE, self.CSV_FILE,
                         self.COUNT_FILE, self.SESSION_FILE]:
            try:
                with open(filename, "w") as f:
                    f.write("")
            except OSError:
                pass

        self.run_count     = 0
        self.session_id    = 0
        self._session_runs = []
        self._ensure_csv_header()

        print("BlackBox: All data cleared. Run count reset to 0.")

    def csv_instructions(self):
        """Print step-by-step instructions for downloading and opening the CSV."""
        print("\n" + "=" * 40)
        print("  HOW TO EXPORT CSV TO EXCEL")
        print("=" * 40)
        print("1. Connect hub to computer via USB")
        print("2. Open Chrome → code.pybricks.com")
        print("3. Click the Bluetooth connect icon")
        print("4. Click the Files icon in the left sidebar")
        print("5. Find blackbox.csv in the file list")
        print("6. Click Download")
        print("7. Open in Excel or Google Sheets")
        print("")
        print("CSV Columns:")
        print("  run, session, mission, move,")
        print("  target, error, abs_error,")
        print("  status, speed, elapsed_ms")
        print("")
        print("Suggested charts in Excel:")
        print("  - Line chart: error over run number")
        print("  - Bar chart: avg error per mission")
        print("  - Scatter: error vs elapsed_ms (battery drain)")
        print("=" * 40 + "\n")

    # ────────────────────────────────────────────────────────────────────────
    # PRIVATE WRITE METHODS
    # ────────────────────────────────────────────────────────────────────────

    def _write_txt(self, report_card, total_err, avg_err, bias, bias_dir, elapsed):
        """Write one run's data to the human-readable text log."""
        try:
            with open(self.TXT_FILE, "a") as f:
                f.write("=" * 36 + "\n")
                f.write("Run #{} | Session {} | {}\n".format(
                    self.run_count, self.session_id, self.current_mission))
                f.write("Speed: {}mm/s | Time: {}ms\n".format(
                    self.current_speed, elapsed))
                f.write("=" * 36 + "\n")

                # Individual move results
                f.write("{:15} | {:7} | {:6} | {}\n".format(
                    "MOVE", "TARGET", "ERROR", "STATUS"))
                f.write("-" * 36 + "\n")

                for name, target, err in report_card:
                    status = "OK" if abs(err) < 0.8 else "CHECK"
                    f.write("{:15} | {:7.1f} | {:+6.2f} | {}\n".format(
                        name, target, err, status))

                # Summary statistics
                f.write("=" * 36 + "\n")
                f.write("Total Error:  {:.2f} deg\n".format(total_err))
                f.write("Avg Error:    {:.2f} deg\n".format(avg_err))
                f.write("Net Bias:     {:+.2f} deg ({})\n".format(bias, bias_dir))
                f.write("=" * 36 + "\n\n")

        except OSError as e:
            print("BlackBox ERROR writing txt: {}".format(e))

    def _write_csv(self, report_card, elapsed):
        """
        Write one run's data to the CSV file.

        CSV columns:
            run, session, mission, move, target, error,
            abs_error, status, speed, elapsed_ms
        """
        try:
            with open(self.CSV_FILE, "a") as f:
                for name, target, err in report_card:
                    status   = "OK" if abs(err) < 0.8 else "CHECK"
                    abs_err  = abs(err)

                    row = "{},{},{},{},{:.2f},{:.4f},{:.4f},{},{},{}\n".format(
                        self.run_count,           # run number (all-time)
                        self.session_id,          # session (power cycle)
                        self.current_mission,     # mission name
                        name,                     # move name
                        float(target),            # target heading
                        err,                      # signed error
                        abs_err,                  # absolute error
                        status,                   # OK or CHECK
                        self.current_speed,       # mm/s
                        elapsed,                  # ms since mission start
                    )
                    f.write(row)

        except OSError as e:
            print("BlackBox ERROR writing csv: {}".format(e))

    def _ensure_csv_header(self):
        """Write CSV header row if the file doesn't exist or is empty."""
        try:
            with open(self.CSV_FILE, "r") as f:
                first = f.read(1)
                if first:
                    return   # file exists and has content — header already there
        except OSError:
            pass   # file doesn't exist yet — write header below

        try:
            with open(self.CSV_FILE, "w") as f:
                f.write("run,session,mission,move,target,error,"
                        "abs_error,status,speed,elapsed_ms\n")
        except OSError as e:
            print("BlackBox ERROR writing csv header: {}".format(e))

    # ────────────────────────────────────────────────────────────────────────
    # PRIVATE UTILITY METHODS
    # ────────────────────────────────────────────────────────────────────────

    def _load_int(self, filename, default=0):
        """Load a single integer from a file. Returns default if file not found."""
        try:
            with open(filename, "r") as f:
                return int(f.read().strip())
        except (OSError, ValueError):
            return default

    def _save_int(self, filename, value):
        """Save a single integer to a file."""
        try:
            with open(filename, "w") as f:
                f.write(str(value))
        except OSError as e:
            print("BlackBox ERROR saving {}: {}".format(filename, e))
