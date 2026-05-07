from pybricks.tools import hub_menu

# ============================================================
# menu.py — FLL 2026-2027 Mission Selector
#
# HOW TO USE:
#   1. Press F5 on THIS file (menu.py) to upload to the hub
#   2. Pybricks will automatically download all mission files
#   3. Use LEFT / RIGHT to scroll through missions
#   4. Press CENTER to launch the selected mission
#   5. When the mission finishes, press the CENTER button
#      on the hub to restart the menu
#
# TO ADD A MISSION:
#   1. Create a new file e.g. mission6.py
#   2. Add "6" to the hub_menu() call below
#   3. Add an elif for selected == "6"
#   4. Re-upload menu.py — it will pull in the new file too
#
# REMEMBER: Any time you change a mission file, re-upload
# menu.py so the hub gets the latest version of everything.
# ============================================================

selected = hub_menu("1", "2", "3", "4", "5")

if selected == "1":
    import mission1
elif selected == "2":
    import mission2
elif selected == "3":
    import mission3
elif selected == "4":
    import mission4
elif selected == "5":
    import mission5
