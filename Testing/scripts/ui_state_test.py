from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore


interface = RedisInterface(host="localhost")
interface.initialize()
server = KeyValueStore(interface)

print(server['UI_STATE'].read())