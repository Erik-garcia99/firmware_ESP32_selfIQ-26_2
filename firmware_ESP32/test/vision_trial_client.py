"""Send ONE explicitly requested local pilot action; never a timed scene script.

Requires paho-mqtt>=2,<3. Each request uses a new UUID. Subscribes before
publishing, verifies correlation and appends an audit JSONL if --log is given.
No credentials are embedded. Does not alter cloud calibration or publish stock.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
import uuid


def main():
    import paho.mqtt.client as mqtt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--broker', required=True)
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--branch', required=True, type=uuid.UUID)
    parser.add_argument('--node', required=True, type=uuid.UUID)
    parser.add_argument('--roi', required=True, type=uuid.UUID)
    parser.add_argument('--session', required=True, type=uuid.UUID)
    parser.add_argument('--action', choices=('status', 'empty', 'full', 'sample', 'cleanup'), required=True)
    parser.add_argument('--confirm-cleanup', action='store_true',
                        help='Explicit permission to remove the experimental reference only')
    parser.add_argument('--discard-current', action='store_true',
                        help='Also permit discarding the reference matching --session and --roi')
    parser.add_argument('--condition', default='unlabelled')
    parser.add_argument('--log', type=Path)
    parser.add_argument('--timeout', type=float, default=30)
    args = parser.parse_args()
    if args.action == 'cleanup' and not args.confirm_cleanup:
        parser.error('cleanup requires --confirm-cleanup')
    if args.discard_current and args.action != 'cleanup':
        parser.error('--discard-current requires --action cleanup and --confirm-cleanup')
    base = f'selfiq/{args.branch}/{args.node}/shelf_cam'
    req = str(uuid.uuid4())
    command = {
        'type': 'vision.experimental', 'confirm_experimental': True,
        'req_id': req, 'session_id': str(args.session), 'roi_id': str(args.roi),
        'action': args.action,
    }
    if args.action == 'cleanup':
        command['confirm_cleanup'] = True
        if args.discard_current:
            command['confirm_discard_current'] = True
    done = threading.Event()
    messages = []
    error = []
    def record(kind, value):
        event = {'at': datetime.now(timezone.utc).isoformat(),
                 'condition': args.condition, 'kind': kind, 'data': value}
        if args.log:
            with args.log.open('a') as output:
                output.write(json.dumps(event, ensure_ascii=False) + '\n')
        print(json.dumps(event, ensure_ascii=False), flush=True)
    def connected(client, userdata, flags, code, properties):
        if code.is_failure:
            error.append(str(code)); done.set(); return
        client.subscribe(base + '/experimental/result', qos=1)
    def subscribed(client, userdata, mid, codes, properties):
        if any(code.is_failure for code in codes):
            error.append('subscription_refused'); done.set(); return
        record('command', command)
        client.publish(base + '/command', json.dumps(command), qos=1, retain=False)
    def message(client, userdata, received):
        if received.retain:
            return
        try:
            value = json.loads(received.payload)
        except (ValueError, UnicodeError):
            return
        if (value.get('req_id') != req or value.get('session_id') != str(args.session)
                or value.get('roi_id') != str(args.roi)):
            return
        record('result', value)
        messages.append(value)
        if value.get('status') != 'measurement':
            done.set()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id='selfiq-vision-test-' + uuid.uuid4().hex[:12])
    client.on_connect, client.on_subscribe, client.on_message = connected, subscribed, message
    try:
        client.connect(args.broker, args.port, keepalive=30)
        client.loop_start()
        if not done.wait(args.timeout):
            raise SystemExit('Timeout: outcome unknown; do not assume capture/persistence succeeded.')
        if error:
            raise SystemExit(str(error))
        final = messages[-1] if messages else {}
        acceptable = {'status': {'uncalibrated', 'empty_ready', 'ready'},
                      'empty': {'empty_captured'}, 'full': {'full_captured'},
                      'sample': {'sample_done'},
                      'cleanup': {'cleanup_completed', 'cleanup_no_reference'}}
        if final.get('status') not in acceptable[args.action]:
            raise SystemExit('Device refused action: ' + str(final.get('status')))
        if args.action == 'sample':
            valid = {m['sample_index'] for m in messages
                     if m.get('status') == 'measurement' and m.get('valid')}
            if valid != {1, 2, 3}:
                raise SystemExit('Incomplete sample: expected three verified measurement messages.')
    finally:
        client.disconnect()
        client.loop_stop()


if __name__ == '__main__':
    main()
