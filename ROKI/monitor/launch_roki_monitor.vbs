Option Explicit

Dim shell
Set shell = CreateObject("WScript.Shell")
shell.Environment("Process")("PYTHONPATH") = "C:\lr-project\.tools"
shell.Run """C:\Users\mrand\AppData\Local\Programs\Python\Python312\pythonw.exe"" ""C:\lr-project\ROKI\monitor\roki_monitor.py""", 0, False
