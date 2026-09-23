"""Read-only Pollen Robotics model adapter. Never sends robot/motor commands."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from urllib.request import urlopen

MODEL_DIR = Path(__file__).parent / 'models' / 'pollen_anyskin_slip'
HASHES = {
    'model.safetensors': 'eeb1603405226d0f039fa33615ef987b473e7c86132f436594b44f90ce4c8b9d',
    'params.json': 'b80cdf2e5bab6b612c6c85b9b156284b4f7d286bb1e529c402a7fdde887b5f37',
}


def vector(values):
    if not isinstance(values, list) or len(values) != 15:
        raise ValueError('15 XYZ channels required; legacy TXY records are not accepted')
    if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
        raise ValueError('non-finite model input')
    return values


def corrected_input(sensor, now):
    stamp = sensor.get('received_pc_monotonic_s')
    if stamp is None or not math.isfinite(stamp) or not 0 <= now - stamp <= 0.25:
        raise ValueError('stale or absent XYZ sample')
    xyz, zero = vector(sensor.get('xyz')), vector(sensor.get('baseline'))
    # Exact duplicate magnetometers are not five independent measurements.
    groups = [xyz[i:i + 3] for i in range(0, 15, 3)]
    if any(all(abs(a - b) < 1e-4 for a, b in zip(groups[i], groups[j]))
           for i in range(5) for j in range(i)):
        raise ValueError('duplicate magnetometer channels; validate sensor firmware/wiring')
    return [v - b for v, b in zip(xyz, zero)]


class PollenSlipModel:
    def __init__(self, directory=MODEL_DIR):
        import torch
        from safetensors.torch import load_file

        directory = Path(directory)
        for name, digest in HASHES.items():
            if hashlib.sha256((directory / name).read_bytes()).hexdigest() != digest:
                raise ValueError(f'Unexpected model artifact: {name}')
        params = json.loads((directory / 'params.json').read_text(encoding='utf-8'))
        self.torch = torch
        torch.set_num_threads(1)
        self.mean = torch.tensor(vector(params['scaler']['mean']), dtype=torch.float32)
        self.scale = torch.tensor(vector(params['scaler']['scale']), dtype=torch.float32)
        if not bool((self.scale > 0).all()):
            raise ValueError('Invalid scaler')
        model_params = params['model']
        if any(model_params[k] != v for k, v in dict(input_size=15, hidden_size=128,
                lstm_hidden_size=128, output_size=1, nlayers=1).items()):
            raise ValueError('Unsupported model architecture')
        # Architecture matches upstream lstm.py (Apache-2.0); no activation
        # between fc and out, sequence length one, fresh hidden state each call.
        class Network(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.lstm = torch.nn.LSTM(15, 128, 1, batch_first=True)
                self.fc = torch.nn.Linear(128, 128)
                self.out = torch.nn.Linear(128, 1)

            def forward(self, x):
                sequence, _ = self.lstm(x)
                return self.out(self.fc(sequence[:, -1, :]))

        self.model = Network()
        self.model.load_state_dict(load_file(str(directory / 'model.safetensors')), strict=True)
        self.model.eval()

    def score(self, baseline_subtracted_xyz):
        torch = self.torch
        x = torch.tensor(vector(baseline_subtracted_xyz), dtype=torch.float32)
        with torch.inference_mode():
            result = torch.sigmoid(self.model(((x - self.mean) / self.scale).reshape(1, 1, 15)))
        value = float(result.item())
        if not math.isfinite(value):
            raise ValueError('Non-finite model output')
        return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8765/command_status')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).parent / 'slip_model_reports' / f'{time.time_ns()}.jsonl')
    args = parser.parse_args()
    model = PollenSlipModel()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f'SHADOW ONLY: no reinforcement or motor commands. Output: {args.output}', flush=True)
    last = {}
    last_display = 0
    with args.output.open('w', encoding='utf-8') as output:
        try:
            while True:
                try:
                    with urlopen(args.url, timeout=1.0) as response:
                        status = json.load(response)
                    snapshot = status.get('slip_model_input', {})
                    if snapshot.get('schema') != 'anyskin_txyz_model_v1':
                        raise ValueError('Restart updated controller: corrected XYZ telemetry missing')
                    for index, sensor in enumerate(snapshot['sensors']):
                        token = (snapshot['calibration_epoch'], sensor['received_pc_monotonic_s'])
                        if last.get(index) == token:
                            continue
                        last[index] = token
                        record = dict(sensor=index + 1, received=token[1], calibration_epoch=token[0],
                                      observed_pc_monotonic_s=time.monotonic(),
                                      contact=bool(status.get('contact_confirmed')), shadow_only=True)
                        try:
                            values = corrected_input(sensor, time.monotonic())
                            record.update(score=model.score(values), input_xyz_delta=values,
                                          state='unvalidated_model_score')
                        except ValueError as exc:
                            record.update(score=None, state=str(exc))
                        output.write(json.dumps(record, allow_nan=False) + '\n')
                        if time.monotonic() - last_display >= 1:
                            print(record, flush=True)
                            last_display = time.monotonic()
                    output.flush()
                except (OSError, ValueError, KeyError) as exc:
                    print(f'Slip inference unavailable: {exc}', flush=True)
                    time.sleep(1)
                time.sleep(0.05)
        except KeyboardInterrupt:
            print('Shadow recording stopped. No motor command was sent.')


if __name__ == '__main__':
    main()
