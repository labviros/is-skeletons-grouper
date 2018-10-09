import json
from is_wire.core import Channel, Subscription, Message
from google.protobuf.struct_pb2 import Struct

channel = Channel(json.load(open('../conf/options.json', 'r'))['broker_uri'])
subscription = Subscription(channel)

groups_id = [0]

config = Struct()
config["max_error"] = 20.0
config["min_score"] = 0.5

for group_id in groups_id:
  topic = 'SkeletonsGrouper.{}.Config'.format(group_id)
  msg = Message(content=config)
  msg.reply_to = subscription
  channel.publish(msg, topic=topic)

while True:
  msg = channel.consume()
  print(msg)
