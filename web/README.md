# Portal do Braço (web dashboard)

Página interna do time: telemetria ao vivo do myCobot 280, documentação do repositório e acompanhamento dos sprints, tudo num lugar só.

| Página | O que mostra | De onde vem |
|---|---|---|
| **Início** | resumo do braço, sprint atual, atalhos | as outras páginas |
| **Braço** | modelo 3D, ângulos com limites soft/firmware, temperaturas, pose cartesiana, histórico de 60 s, botão STOP, tabela de comandos, limites e PID de fábrica | WebSocket `/ws/telemetry` (pymycobot) + `js/specs.js` |
| **Documentação** | todos os `.md` de `docs/` (e `docker/README.md`), renderizados | `GET /api/docs` |
| **Sprints** | linha do tempo dos milestones, contagem regressiva, um card por issue `SPRINT N` com as sub-issues | API do GitHub, cache de 5 min |

> **Somente leitura.** O painel nunca liga os servos, move juntas ou grava configurações. O único comando enviado ao braço é `mc.stop()`, pelo botão STOP.

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
| `ARM_PORT` | `/dev/ttyTHS1` | porta serial |
| `ARM_BAUD` | `1000000` | baud rate |
| `GITHUB_TOKEN` | — | opcional; aumenta o limite da API do GitHub (60 req/h sem token) |
| `GITHUB_REPO` | `SEAME-pt/SEA-ME_Team2_braco` | repositório dos sprints |

## Estrutura

```
web/
  server/app.py      FastAPI: rotas /api/*, WebSocket, arquivos estáticos
  server/arm.py      ArmSource (pymycobot) e MockArm
  server/docs.py     lista e lê os Markdown de docs/
  server/github.py   milestones, issues SPRINT e sub-issues
  static/            HTML/CSS/JS puro, sem build (three.js, marked e highlight.js via CDN)
```

## Como manter

- **Novo documento:** coloque o `.md` em `docs/` e ele aparece sozinho.
- **Novo sprint:** crie a issue com título `SPRINT N | Nome` e adicione as tarefas como sub-issues no GitHub.
- **Mudou limite, PID ou comando em `mycobot.hpp`:** atualize `static/js/specs.js`.

## API

| Rota | Descrição |
|---|---|
| `GET /api/status` | último snapshot do braço |
| `WS /ws/telemetry` | snapshot a cada 200 ms |
| `POST /api/stop` | `mc.stop()` |
| `GET /api/docs` | lista de documentos |
| `GET /api/docs/{path}` | Markdown de um documento |
| `GET /api/sprints?refresh=1` | dados dos sprints (`refresh` ignora o cache) |
