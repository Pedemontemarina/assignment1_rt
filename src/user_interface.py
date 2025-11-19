'''
✓Implement a simple textual interface to retrieve the user command 
(i.e., you can use cin (c++) or input (python). The user should be able to 
select the robot they want to control (turtle1 or turtle2), and the velocity 
of the robot. 

✓The command should be sent for 1 second, and then the robot should 
stop, and the user should be able again to insert the command. 

1) chiedere all'utente quale tartaruga controllare

2) utente inserisce velocità 

3) invio il comando per 1s e poi la tarta si ferma

4) l'interfaccia torna all'utente

inizializza nodo ROS
crea publisher su /turtle1/cmd_vel e /turtle2/cmd_vel

loop infinito:
    chiedi all’utente quale tartaruga controllare
    chiedi velocità lineare e angolare
    crea messaggio Twist
    pubblica comando
    aspetta 1 secondo
    ferma la tartaruga (Twist a zero)
'''

while True: 

    turtle_name = input("Seleziona la tartaruga da controllare (turtle1/turtle2): ")
    linear = float(input("Velocità lineare: "))
    angular = float(input("Velocità angolare: "))

    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular

    publisher.publish(msg)
    time.sleep(1.0)

    msg = Twist()  # tutto a zero
    publisher.publish(msg)

