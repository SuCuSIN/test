"""Exercise the worker's actual bus-call expression without opening hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class MotionProfileTest(unittest.TestCase):
    def test_both_owners_use_configured_profile_and_one_write(self):
        tree = ast.parse(Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py').read_text(encoding='utf-8'))
        controller = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                          and n.name == 'RosPositionGripperController')
        worker = next(n for n in controller.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_worker_loop')
        calls = [n for n in ast.walk(worker) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Attribute) and n.func.attr == 'sync_move']
        self.assertEqual(len(calls), 1)
        expression = compile(ast.Expression(body=calls[0]), '<worker motion call>', 'eval')
        for reinforcement in (False, True):
            for speed in (2919, 120):
                with self.subTest(reinforcement=reinforcement, speed=speed):
                    client = Mock()
                    owner = SimpleNamespace(client=client, args=SimpleNamespace(speed=speed, acc=22))
                    eval(expression, {'self': owner, 'target': (1748, 2250),
                                      'regrasp_move': reinforcement})
                    client.sync_move.assert_called_once_with(1748, 2250, speed, 22)


if __name__ == '__main__':
    unittest.main()
