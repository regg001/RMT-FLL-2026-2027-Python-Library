
"""
Pull blackbox.csv from hub and save locally with timestamp.
Run with: python pull_data.py
"""

import os
from datetime import datetime

def pull_blackbox():
    """Read blackbox.csv from hub storage and save locally."""
    try:
        # Read from hub (connected via USB/Pybricks)
        with open("blackbox.csv", "r") as f:
            data = f.read()
        
        # Create data/ folder if needed
        os.makedirs("data", exist_ok=True)
        
        # Get current timestamp
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        readable_date = now.strftime("%A, %B %d, %Y at %I:%M:%S %p")
        
        # Create header with metadata
        header = f"# Pulled: {readable_date}\n"
        header += f"# Timestamp: {timestamp}\n"
        header += f"# All-time rows: {len(data.splitlines()) - 1}\n\n"
        
        local_file = f"data/blackbox_{timestamp}.csv"
        
        # Write header + CSV data
        with open(local_file, "w") as f:
            f.write(header)
            f.write(data)
        
        print(f"✓ Saved: {local_file}")
        print(f"✓ Pulled: {readable_date}")
        print(f"✓ Rows: {len(data.splitlines()) - 1}")  # count minus header
        
    except FileNotFoundError:
        print("ERROR: blackbox.csv not found on hub")
        print("Run at least one mission first.")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    pull_blackbox()