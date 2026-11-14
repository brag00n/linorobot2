@echo off
rem Lance le serveur MCP HTTP en lecture seule (ros2-analysis) pour Aria. Appele par la tache planifiee « Bamboo-MCP-HTTP » (ouverture de session).
rem Le jeton n'est PAS ici : il est lu dans %USERPROFILE%\.ros2_mcp_token (cree au premier lancement, hors depot).
rem Journal : %LOCALAPPDATA%\Bamboo\mcp_http.log (remis a zero a chaque demarrage, 1 Mo au plus par execution).
setlocal
set "ROOT=%~dp0..\..\"
set "ROS2_BRIDGE_URL=wss://192.168.1.164:8443/rosbridge"
set "ROS2_MCP_PORT=8790"
set "ROS2_MCP_HOST=0.0.0.0"
if not exist "%LOCALAPPDATA%\Bamboo" mkdir "%LOCALAPPDATA%\Bamboo"
cd /d "%ROOT%"
"%ROOT%.venv\Scripts\python.exe" -m ros_control.mcp.http_server > "%LOCALAPPDATA%\Bamboo\mcp_http.log" 2>&1
