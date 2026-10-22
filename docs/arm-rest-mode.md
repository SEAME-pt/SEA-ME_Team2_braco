# Modo "rest" do braço e comandos básicos

## Por que mudou

Rodar `park.py` direto no host (versão antiga, serial direta) enquanto o `arm_driver` estava ativo dava
`SerialException: ... multiple access on port?`. Para contornar, os containers eram derrubados, e sem o
`arm_driver` ninguém lia a temperatura: o braço esfriava sem monitoramento.

## O que mudou

| Onde | Mudança |
|---|---|
| `arm_driver.cpp` | Novo **modo rest** (somente leitura): recusa movimento, gripper e `power_on`, mas continua publicando `/joint_states` e `/arm/servo_temperatures`. Serviço `/arm/set_read_only` (`std_srvs/SetBool`), tópico `/arm/read_only` (latched) e parâmetro `read_only`. `STOP` e `release_servos` continuam funcionando. |
| `ros2_ws/src/mycobot_cpp/srcs/mycobot.cpp` | A porta serial é aberta com `TIOCEXCL`: um segundo processo (não-root) recebe "Device or resource busy" na hora, em vez de corromper a leitura. |
| `park` (C++, `braco_bringup`) | Sai do rest, vai para a pose de park, solta os servos e **entra em rest**. O driver segue vivo medindo temperatura. |
| `zero` (C++, `braco_bringup`) | Sai do rest antes de ligar os servos e mover. |
| `arm_moves.cpp` | Novo `setRest(on)` (cliente do serviço acima). |
| `Makefile` (repo root) | Comandos `park`, `zero`, `rest`, `resume`, `temps` (rodam dentro do container `real`; eram `arm-*` antes da #54). |

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

- **Nunca abra a serial fora do `arm_driver`** (as ferramentas avulsas de `mycobot_cpp`, pymycobot no host). Use os targets acima.
- Em rest, o joystick e o `ArmCommand` são ignorados (aviso no log do driver a cada 5 s).
- Um servo a ≥ 65 °C (`TEMP_HOT_C`) põe o braço em rest **sozinho**: o `temperature_monitor` chama
  `/arm/set_read_only true` uma vez por episódio (até todas as juntas ficarem abaixo de 65 °C) e o LCD mostra
  `REST: SERVO HOT` / `J4 66 C cooling`. A volta é sempre manual (`make resume`); um `make resume` durante o
  mesmo episódio não é desfeito (serve para estacionar um braço quente de propósito). Desligável com o
  parâmetro `auto_rest_on_hot`. Ver [ADR-0003](ADR-0003-repouso-por-temperatura.md).
- `zero` e `park` pegam o controle do braço à força (`take_control("script", force=true)`) e devolvem no fim:
  funcionam com o joystick conectado ou a página no controle. Ver [ADR-0002](ADR-0002-controle-unico.md).
- O `arm_routine.sh` (entrypoint do container `real`) não move o braço ao parar: use `make down PARK=1`, que roda `park` antes de desligar e deixa o braço em rest.
- `ros2 topic echo /arm/read_only` mostra o estado atual.

## Como testar

1. `make up`
2. `make park`, depois `make temps` retorna `valid: true`.
3. Joystick não move o braço; `make resume` e ele volta a mover.
4. Com o driver ligado, uma ferramenta avulsa (`make -C ros2_ws/src/mycobot_cpp temps_lcd && ros2_ws/src/mycobot_cpp/temps_lcd`)
   falha com "Device or resource busy".
5. Servo quente simulado: com o sistema rodando, uma leitura falsa a 66 °C basta (a leitura real seguinte,
   2 s depois, encerra o episódio):
   `make run CMD='ros2 topic pub --once /arm/servo_temperatures braco_interfaces/msg/ServoTemperatures "{celsius: [40, 40, 40, 66, 40, 40], valid: true}"'`.
   O braço entra em rest, o log do monitor e o LCD avisam, e `make resume` faz voltar.
