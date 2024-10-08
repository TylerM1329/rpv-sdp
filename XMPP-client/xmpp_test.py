import xmpp
username = 'rpv'
passwd = '022223'
to = 'roza@0.0.0.0'
msg = "EMER#5231.453#1323.658#10#33.33#200#400"

client = xmpp.Client('0.0.0.0')
client.connect(server=('10.8.0.4', 5222))
client.auth(username, passwd)
client.sendInitPresence()

message = xmpp.Message(to, msg)
message.setAttr('type', 'chat')

client.send(message)