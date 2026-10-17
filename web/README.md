# Portal do Braço (web dashboard)

Página interna do time: telemetria ao vivo do myCobot 280, documentação do repositório e acompanhamento dos sprints, tudo num lugar só.

| Página | O que mostra | De onde vem |
|---|---|---|
| **Início** | resumo do braço, sprint atual, atalhos | as outras páginas |
| **Braço** | modelo 3D, ângulos com limites soft/firmware, temperaturas, pose cartesiana, histórico de 60 s, controle (STOP, Park, Zero e slider por junta), tabela de comandos, limites e PID de fábrica | WebSocket `/ws/telemetry` (pymycobot) + `js/specs.js` |
| **Documentação** | todos os `.md` do repositório, agrupados por tema, renderizados | `GET /api/docs` |
| **Sprints** | linha do tempo dos milestones, contagem regressiva, um card por issue `SPRINT N` com as sub-issues e uma página de detalhes por sprint (`#/sprints/N`: período, estatísticas, evoluções com PRs integrados e tarefas concluídas, pendentes, equipe) | API do GitHub, cache de 5 min |

> **Quase somente leitura.** O painel nunca liga os servos nem grava configurações. Os únicos comandos enviados ao braço são `mc.stop()` (botão STOP) `send_angles()` para as poses Park e Zero (com confirmação) e `send_angle()` por junta, dentro do soft limit e só com “liberar movimento” marcado.

## Rodar

No PC, sem o braço (dados simulados):

```bash
make web-mock          # http://localhost:8000
```

No Jetson, com o braço:

```bash
make web               # http://<ip-do-jetson>:8000 de qualquer PC na rede do laboratório
```

Os dois alvos instalam `web/requirements.txt` no `.venv` do repositório na primeira vez. O `make web` também instala o `requirements.txt` raiz (pymycobot); o `make web-mock` não precisa dele, então funciona em qualquer Python 3.8+ (o `numpy==1.24.4` do `requirements.txt` raiz não instala no Python 3.12).

**A porta serial só aceita um programa por vez.** Pare `joystick_display`, `temps_lcd`, scripts do `playground/` ou o driver ROS antes de `make web`. Se a porta estiver ocupada, o servidor continua no ar e a página mostra "braço desconectado" com o erro; ele tenta reconectar a cada 3 s.

## Simulação no Gazebo (só no PC)

A página pode mostrar e comandar o braço simulado no Gazebo em vez do braço real. O servidor roda **dentro do container `pc-dev`** (onde estão o ROS 2 e o `rclpy`), assina `/joint_states` e publica trajetórias em `/joint_trajectory_controller/joint_trajectory`:

```bash
cd docker
make build          # uma vez: a imagem ganhou fastapi/uvicorn
make web-sim        # http://localhost:8000, com a simulação já rodando no Gazebo
```

Só funciona quando a máquina é o PC. O servidor descobre o papel como o Makefile do `docker/`: `SEAME_ROLE` (dentro dos containers), `COMPOSE_PROFILES` (ambiente ou `docker/.env`) e, sem nenhum dos dois, a CPU (aarch64 = Jetson). Com `ARM_SOURCE=gazebo` no Jetson ele recusa subir; `make web-sim` com `ROLE=jetson` também recusa. Na simulação não há temperaturas nem repouso dos servos: esses campos ficam vazios. Os nomes das juntas e os tópicos assumem o URDF do myCobot 280 e podem ser trocados por `GZ_JOINT_NAMES`.

## Testes

```bash
make web-test
```

Não precisa do braço nem de rede:

- `tests/test_server.py`: rotas HTTP e WebSocket, bloqueio de caminhos fora de `docs/`, agrupamento dos sprints do GitHub (com respostas falsas), cache e o modo simulado.
- `tests/test_emulated_arm.py`: o servidor com o **pymycobot real** falando com um braço emulado num pseudo-terminal. Confere os valores lidos, que **só comandos de leitura** saem pela serial, que o STOP sai mesmo enquanto o braço não responde, e a reconexão. Pulado se o pymycobot não estiver instalado.

O alvo é o Python 3.8 do Jetson. Para testar nessa versão a partir de um PC:

```bash
docker run --rm -v "$PWD":/src:ro python:3.8 bash -c 'cp -r /src /w && cd /w && rm -rf .venv && make web-test'
```

## Variáveis de ambiente

| Variável | Padrão | Uso |
|---|---|---|
| `ARM_MOCK` | — | `1` usa dados simulados |
| `ARM_SOURCE` | — | `gazebo` usa o braço simulado via ROS 2 (só PC, ver acima) |
| `GZ_JOINT_NAMES` | nomes do URDF do myCobot 280 | juntas em `/joint_states`, separadas por vírgula |
| `ARM_PORT` | `/dev/ttyTHS1` | porta serial |
| `ARM_BAUD` | `1000000` | baud rate |
| `GITHUB_TOKEN` | — | opcional; aumenta o limite da API do GitHub (60 req/h sem token) |
| `GITHUB_REPO` | `SEAME-pt/SEA-ME_Team2_braco` | repositório dos sprints |

## Estrutura

```
web/
  server/app.py      FastAPI: rotas /api/*, WebSocket, arquivos estáticos
  server/arm.py      ArmSource (pymycobot) e MockArm
  server/docs.py     descobre, agrupa e lê todos os Markdown do repositório
  server/github.py   milestones, issues SPRINT e sub-issues
  static/            HTML/CSS/JS puro, sem build (three.js, marked e highlight.js via CDN)
```

## Como manter

- **Novo documento:** coloque o `.md` em qualquer pasta do repositório e ele aparece sozinho (o grupo é definido em `server/docs.py`).
- **Novo sprint:** crie a issue com título `SPRINT N | Nome` e adicione as tarefas como sub-issues no GitHub.
- **Mudou limite, PID ou comando em `mycobot.hpp`:** atualize `static/js/specs.js`.

## API

| Rota | Descrição |
|---|---|
| `GET /api/status` | último snapshot do braço |
| `WS /ws/telemetry` | snapshot a cada 200 ms |
| `POST /api/stop` | `mc.stop()` |
| `POST /api/pose/{park\|zero}?speed=` | `mc.send_angles(pose, speed)`; park depois chama `release_all_servos()` ao chegar (braço em repouso); 409 se já estiver em repouso |
| `POST /api/wake` | `mc.power_on()` + `send_angles(zero, speed)`: sai do repouso indo para o zero |
| `POST /api/joint/{1-6}` `{angle, speed}` | `mc.send_angle(n, angle, speed)`, dentro do soft limit |
| `GET /api/docs` | lista de documentos |
| `GET /api/docs/{path}` | Markdown de um documento |
| `GET /api/sprints?refresh=1` | dados dos sprints (`refresh` ignora o cache) |
