Skeletons Grouper Service
===

Service to group skeletons detections from multiples cameras with overlapping.

Streams
---

| Name | Input (Topic/Message) | Output (Topic/Message) | Description | 
| ---- | --------------------- | ---------------------- | ----------- |
| SkeletonsGrouper.(GROUP_ID).Localize | **SkeletonsDetector.(ID).Detection** [ObjectAnnotations] | **SkeletonsGrouper.(GROUP_ID).Localization** [ObjectAnnotations] | Groups and localizes skeletons detections from different sources and publishes three-dimensional skeletons in the given frame. |

RPCs
---
| Service | Request | Reply | Description |
| ------- | ------- | ------| ----------- |
| SkeletonsGrouper.(GROUP_ID).Config | [google::protobuf::Struct](https://github.com/protocolbuffers/protobuf/blob/master/src/google/protobuf/struct.proto) | [google::protobuf::Empty](https://github.com/protocolbuffers/protobuf/blob/master/src/google/protobuf/empty.proto) | Allows user to configure the maximum error (**"max_error"**) used on the matching process and the minimum score (**"min_score"**) per skeleton joint received. Both parameters are represented by a floating point. The maximum error is given in pixels and minimum score is an unity value. You can find a simple example on [etc/rpc_example/configure.py](https://github.com/labviros/is-skeletons-grouper/blob/master/etc/rpc_example/configure.py) showing how to make a request to this service. |