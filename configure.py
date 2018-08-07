import json
from is_wire.core import Channel, Subscription, Message
from google.protobuf.struct_pb2 import Struct

channel = Channel(json.load(open('options.json', 'r'))['broker_uri'])
subscription = Subscription(channel)

groups = [[0,1,2,3,4], [0,2,4,6], [5,6,7]]

config = Struct()
config["max_error"] = 20.0
config["min_score"] = 0.5

for group in groups:
  topic = 'Skeletons.Localization.{}.Config'.format('.'.join(map(str, group)))
  print(topic)
  msg = Message(content=config)
  msg.reply_to = subscription
  channel.publish(msg, topic=topic)

while True:
  msg = channel.consume()
  print(msg)
