If you run this on PC please use `python pc-camera.py` file.

## Run on Raspberry Pi

`python main.py`

## Run on PC

`python pc-camera.py`

curl https://pyenv.run | bash

export PATH="~/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"

source ~/.bashrc
pyenv install 3.6
pyenv versions
pyenv local 3.6.15

pip install pymavlink==2.4.8
