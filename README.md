# User Guide:
## python venv
Simplest way is to make a python venv for this is to input into terminal line by line


On linux:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
On windows consider upgrading to linux and/or:
```bash
Set-ExecutionPolicy Unrestricted -Force
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```


## Running
Then run the code in the folder terminal

```bash
panel serve gui.py --autoreload --show
```
-- autoreload is not necessary, but nice if you want to update the code easily
