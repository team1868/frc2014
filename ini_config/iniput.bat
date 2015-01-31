ftp -i -s:"%~f0"&GOTO:EOF
!:--- OK TO SEE INVALID COMMAND,PROCEED ---
open 10.18.68.2
root
admin 
!:--- FTP commands below here ---
ascii
put robot.ini
disconnect
bye
