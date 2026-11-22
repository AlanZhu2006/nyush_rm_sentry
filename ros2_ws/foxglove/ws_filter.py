"""Relay visualization from a read-only bridge and publish fixed ROS commands."""

import argparse
import asyncio
import json
import signal
import struct

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed, InvalidHandshake


PROTOCOLS = ['foxglove.sdk.v1', 'foxglove.websocket.v1']
OPERATOR_SCHEMAS = {
    '/sentry/operator/command': ('std_msgs/msg/String', String),
    '/sentry/operator/drive': ('std_msgs/msg/String', String),
    '/sentry/operator/goal': ('geometry_msgs/msg/PoseStamped', PoseStamped),
    '/sentry/operator/initialpose': ('geometry_msgs/msg/PoseWithCovarianceStamped', PoseWithCovarianceStamped),
}
VIEW_OPERATIONS = {'subscribe', 'unsubscribe', 'subscribeConnectionGraph', 'unsubscribeConnectionGraph'}


class PublishPolicy:
    def __init__(self, publishers):
        self.publishers = publishers
        self.channels = {}

    def filter(self, frame):
        if isinstance(frame, bytes):
            if 5 < len(frame) <= 4096 and frame[0] == 1:
                channel, = struct.unpack_from('<I', frame, 1)
                if channel in self.channels:
                    topic = self.channels[channel]
                    try:
                        message = deserialize_message(frame[5:], OPERATOR_SCHEMAS[topic][1])
                    except (RuntimeError, ValueError, TypeError):
                        return None
                    self.publishers[topic].publish(message)
            return None

        try:
            message = json.loads(frame)
        except (ValueError, RecursionError):
            return None
        if not isinstance(message, dict) or not isinstance(message.get('op'), str):
            return None
        op = message['op']
        if op in VIEW_OPERATIONS:
            return json.dumps(message)
        if op == 'advertise' and isinstance(message.get('channels'), list):
            for channel in message['channels']:
                if not isinstance(channel, dict) or len(self.channels) >= 24:
                    continue
                channel_id = channel.get('id')
                topic = channel.get('topic')
                if not isinstance(topic, str) or topic not in OPERATOR_SCHEMAS:
                    continue
                schema, _ = OPERATOR_SCHEMAS[topic]
                if (channel.get('schemaName') in (schema, schema.replace('/msg/', '/'))
                        and channel.get('encoding') == 'cdr' and type(channel_id) is int
                        and 0 <= channel_id < 2**32 and channel_id not in self.channels):
                    self.channels[channel_id] = topic
        if op == 'unadvertise' and isinstance(message.get('channelIds'), list):
            for channel_id in message['channelIds']:
                if type(channel_id) is int:
                    self.channels.pop(channel_id, None)
        return None


def server_frame(frame):
    if isinstance(frame, str):
        message = json.loads(frame)
        if isinstance(message, dict) and message.get('op') == 'serverInfo':
            capabilities = message.get('capabilities', [])
            message['capabilities'] = sorted(set(capabilities) | {'clientPublish'})
            message['supportedEncodings'] = ['cdr']
            return json.dumps(message)
    return frame


async def forward(source, destination, filter_frame):
    try:
        async for frame in source:
            forwarded = filter_frame(frame)
            if forwarded is not None:
                await destination.send(forwarded)
    except ConnectionClosed:
        pass


async def run(port, upstream_port, publishers):
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)

    async def handle(client):
        try:
            async with connect(
                f'ws://127.0.0.1:{upstream_port}', subprotocols=[client.subprotocol],
                max_size=24 * 1024 * 1024, open_timeout=4, proxy=None,
            ) as bridge:
                policy = PublishPolicy(publishers)
                tasks = [asyncio.create_task(forward(client, bridge, policy.filter)),
                         asyncio.create_task(forward(bridge, client, server_frame))]
                try:
                    await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                finally:
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
        except (OSError, TimeoutError, InvalidHandshake):
            await client.close(1013, 'Foxglove Bridge is starting')

    async with serve(handle, '127.0.0.1', port, subprotocols=PROTOCOLS, max_size=16384):
        print(f'Foxglove operator listening at ws://127.0.0.1:{port}', flush=True)
        await stopped.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--upstream-port', type=int, default=18766)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sentry_foxglove_gateway', enable_rosout=False, start_parameter_services=False)
    publishers = {topic: node.create_publisher(kind, topic, 20 if topic.endswith('/command') else 1)
                  for topic, (_, kind) in OPERATOR_SCHEMAS.items()}
    try:
        asyncio.run(run(args.port, args.upstream_port, publishers))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
