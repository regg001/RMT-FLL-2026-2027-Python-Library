from blackbox import BlackBox

# ============================================================
# read_log.py — FLL 2026-2027
#
# Run this file with F5 in VSCode to print the full black box
# history to the terminal for post-competition review.
#
# To export CSV to Excel:
#   Run box.csv_instructions() below for step-by-step guide,
#   or download blackbox.csv directly from the hub via
#   code.pybricks.com → Files → blackbox.csv → Download
# ============================================================

box = BlackBox()

# Print full text history to terminal
box.print_history()

# Print CSV download instructions
box.csv_instructions()

# Print this session's summary
box.session_summary()
