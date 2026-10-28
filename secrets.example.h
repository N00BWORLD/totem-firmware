#pragma once
// 이 파일을 secrets.h 로 복사하고, 아래 키를 내 키(무작위 16바이트)로 바꾸세요.
// 짝이 되는 토템 두 대에는 반드시 같은 키를 넣어야 서로 보입니다.
// 키가 같은 토템끼리만 위치를 주고받으므로 키는 남에게 알려주지 마세요. (secrets.h 는 git에 올라가지 않음)
//
// 키 만들기 (PowerShell):
//   $k=New-Object byte[] 16; [Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($k); ($k|%{'0x{0:X2}' -f $_}) -join ', '
// 키 만들기 (macOS/Linux):
//   od -An -tx1 -N16 /dev/urandom | sed 's/ \([0-9a-f]\{2\}\)/0x\1, /g'
static const uint8_t TOTEM_KEY[16] = {0};
