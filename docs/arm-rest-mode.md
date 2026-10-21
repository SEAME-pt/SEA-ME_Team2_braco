# Modo "rest" do braço e comandos básicos

## Por que mudou

Rodar `park.py` direto no host (versão antiga, serial direta) enquanto o `arm_driver` estava ativo dava
`SerialException: ... multiple access on port?`. Para contornar, os containers eram derrubados, e sem o
`arm_driver` ninguém lia a temperatura: o braço esfriava sem monitoramento.

## O que mudou

| Onde | Mudança |
|---|---|
| `arm_driver.cpp` | Novo **modo rest** (somente leitura): recusa movimento, gripper e `power_on`, mas continua publicando `/joint_states` e `/arm/servo_temperatures`. Serviço `/arm/set_read_only` (`std_srvs/SetBool`), tópico `/arm/read_only` (latched) e parâmetro `read_only`. `STOP` e `release_servos` continuam funcionando. |
| `libs/cpp/srcs/mycobot.cpp` | A porta serial é aberta com `TIOCEXCL`: um segundo processo (não-root) recebe "Device or resource busy" na hora, em vez de corromper a leitura. |
| `park` (C++, `braco_bringup`) | Sai do rest, vai para a pose de park, solta os servos e **entra em rest**. O driver segue vivo medindo temperatura. |
| `zero` (C++, `braco_bringup`) | Sai do rest antes de ligar os servos e mover. |
| `arm_moves.cpp` | Novo `setRest(on)` (cliente do serviço acima). |
| `Makefile` (repo root) | Comandos `park`, `zero`, `rest`, `resume`, `temps` (rodam dentro do container `real`; eram `arm-*` antes da #54). |
| `playground/park.py` | Aviso no topo: não usar com o driver ligado. |

## Como usar (na Jetson, em `docker/`)

```bash
make park      # estaciona, solta os servos e entra em rest (nada precisa ser parado)
make temps     # imprime as temperaturas uma vez (funciona em rest)
make logs     # acompanha os nós (monitor, driver, teleop); o LCD continua atualizando
make resume    # sai do rest (movimento aceito de novo)
make zero      # sai do rest, liga os servos e vai para a pose zero
make rest      # entra em rest sem estacionar (ex.: só travar movimento)
```

Fluxo típico com braço quente: `make park` → esperar → `make temps` (ou olhar o LCD) até
normalizar (< 55 °C) → `make zero` (ou `make resume` se for usar o joystick; os servos continuam soltos
até `arm-zero` ou `power_on`).

## Regras

- **Nunca abra a serial fora do `arm_driver`** (scripts de `playground/`, pymycobot no host). Use os targets acima.
- Em rest, o joystick e o `ArmCommand` são ignorados (aviso no log do driver a cada 5 s).
- O `arm_routine.sh` (entrypoint do container `real`) não move o braço ao parar: use `make down PARK=1`, que roda `park` antes de desligar e deixa o braço em rest.
- `ros2 topic echo /arm/read_only` mostra o estado atual.

## Ainda não existe

- Park automático ao atingir 65 °C (o monitor continua passivo).
- Indicação de "REST" no LCD.

## Como testar

1. `make up`
2. `make park`, depois `make temps` retorna `valid: true`.
3. Joystick não move o braço; `make resume` e ele volta a mover.
4. Com o driver ligado, `python3 playground/park.py` no host falha com "Device or resource busy".
