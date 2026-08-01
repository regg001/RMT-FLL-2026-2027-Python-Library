import gc

from pybricks.tools import hub_menu, wait

# ============================================================
# menu.py — FLL 2026-2027 Mission Selector
#
# HOW TO USE:
#   1. Press F5 on THIS file (menu.py) to upload to the hub
#   2. Pybricks will automatically download all mission files
#   3. Use LEFT / RIGHT to scroll through missions
#   4. Press CENTER to launch the selected mission
#   5. When the mission finishes, the menu shows again
#      automatically — no re-upload needed
#
# TO ADD A MISSION:
#   1. Create a new file e.g. mission6.py (with a main() function)
#   2. Add "6" to the hub_menu() call below
#   3. Add an elif for selected == "6"
#   4. Re-upload menu.py — it will pull in the new file too
#
# REMEMBER: Any time you change a mission file, re-upload
# menu.py so the hub gets the latest version of everything.
# ============================================================

while True:
    selected = hub_menu("1", "2", "3", "4", "5")

    if selected == "1":
        import Run_1
        Run_1.main()
    elif selected == "2":
        import Run_2
        Run_2.main()
    elif selected == "3":
        import Motor_test
        Motor_test.main()

    # Force the previous Robot()'s Motor objects to release their
    # ports now, instead of waiting for MicroPython's GC to get to
    # it eventually — otherwise the next run's Motor(...) call can
    # fail with EBUSY because the old ones are still open.
    gc.collect()

    # Let button hardware settle before hub_menu() reads it again.
    wait(300)
