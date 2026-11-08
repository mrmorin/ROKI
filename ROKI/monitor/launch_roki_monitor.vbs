Option Explicit

Dim shell, fso, here
Set shell = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
here = fso.GetParentFolderName(WScript.ScriptFullName)
shell.Run "pythonw.exe """" & here & "\roki_monitor.py"""", 0, False
