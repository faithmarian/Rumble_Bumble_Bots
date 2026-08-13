@echo off
setlocal
cd /d "%~dp0"

set "VENV=%~dp0..\.venv"
set "PYTHON=%VENV%\Scripts\python.exe"

if not exist "%PYTHON%" (
    py -m venv "%VENV%"
)

"%PYTHON%" -m pip install -r "%~dp0requirements.txt"
"%PYTHON%" -m ipykernel install --user --name mtrn3100-week12 --display-name "Python (MTRN3100 Week 12)"

if exist "C:\ProgramData\anaconda3\Scripts\jupyter-notebook.exe" (
    "C:\ProgramData\anaconda3\Scripts\jupyter-notebook.exe" "%~dp0Path_Generation_4_1_1.ipynb"
) else (
    "%PYTHON%" -m jupyter notebook "%~dp0Path_Generation_4_1_1.ipynb"
)

