# User Guide:
## python venv
Simplest way is to make a python venv for this is to input into terminal line by line

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Running
Then run the code in the folder terminal

```bash
panel serve gui.py --autoreload --show
```