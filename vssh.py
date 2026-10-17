#!/usr/bin/env python3
"""Run a command in the CEMU VM as root via ssh (password auth)."""
import sys
import pexpect

cmd = sys.argv[1]
child = pexpect.spawn(
    f'ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null '
    f'-p 2222 root@127.0.0.1 {cmd!r}',
    encoding='utf-8', timeout=1800,
)
i = child.expect(['[Pp]assword', pexpect.EOF])
if i == 0:
    child.sendline('root')
    child.expect(pexpect.EOF)
print(child.before, end='')
sys.exit(child.exitstatus or 0)
