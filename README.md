# User Guide:
## python venv
Simplest way is to make a python venv for this is to input into terminal line by line

On windows start with this so you can create and enter venv
```bash
Set-ExecutionPolicy Unrestricted -Force
```

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```



## Running
Then run the code in the folder terminal

```bash
panel serve gui.py --autoreload --show
```
