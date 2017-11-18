qdbus org.kde.yakuake /yakuake/tabs setTabTitle 0 "first_tab"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Rcis" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "rcis robocomp/files/innermodel/simpleworld.xml"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Storm" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "rcnode"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Kdevelop" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "kdevelop &"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Joystick" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "cd robocomp/components/robocomp-robolab/components/joystickComp/bin"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Myfirstcomp" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "robotics_project/myfirstcomp/bin/myfirstcomp robotics_project/myfirstcomp/etc/config"

qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession
qdbus org.kde.yakuake /yakuake/tabs setTabTitle $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "Supervisor" 
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.activeSessionId) "robotics_project/supervisor/bin/supervisor robotics_project/supervisor/etc/config"

