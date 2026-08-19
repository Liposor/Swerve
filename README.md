# Swerve 2026 — WPILib + Phoenix 6

Base em Java para simular e evoluir um drivetrain swerve com Phoenix 6, três Limelights,
MegaTag2, histórico de odometria, estados da Superstructure e PathPlanner.

> **Segurança:** IDs, offsets, reduções, massa, momento de inércia, atrito, dimensões e poses das
> câmeras ainda são exemplos. O projeto continua bloqueado no robô real enquanto
> `TunerConstants.HARDWARE_CONFIGURED` for `false`.

## O que está integrado

- Controle field-centric e simulação nativa do swerve Phoenix 6.
- Três Limelights configuráveis: frontal, esquerda e direita.
- Leitura MegaTag2 diretamente por NetworkTables (`botpose_orb_wpiblue`).
- Simulação geométrica de FOV, alcance, latência e IDs de AprilTags visíveis.
- Filtro de confiabilidade antes de cada `addVisionMeasurement`.
- Desvio-padrão XY dinâmico por distância e quantidade de tags; heading vem do Pigeon.
- Histórico circular de pose e velocidades a cada 20 ms por dois segundos.
- Monitor conservador de impacto, bloqueio e possível derrapagem.
- Solver de estados com X-lock durante pontuação ou bloqueio confirmado.
- PathPlanner 2026.1.2 com feedforwards de força por módulo.
- Monitor de intervalo do loop, memória Java, temperatura e saúde/utilização CAN.
- Telemetria limitada em frequência para reduzir CPU, GC e tráfego de NetworkTables.

## Arquivos de configuração

- `config/ConfigSwerve.java`: velocidades, deadband, simulação e perfis da Superstructure.
- `config/ConfigVision.java`: nomes, transforms, FOV e todos os gates das câmeras.
- `config/ConfigLocalization.java`: colisão, histórico, X-lock, PathPlanner e desempenho.
- `generated/TunerConstants.java`: hardware do swerve; deve ser regenerado no Phoenix Tuner X.
- `src/main/deploy/pathplanner/settings.json`: modelo físico usado pelo PathPlanner.

## Limelights e MegaTag2

Os nomes configurados são:

- `limelight-front`
- `limelight-left`
- `limelight-right`

O código envia a pose robot-space da câmera e `robot_orientation_set` em cada atualização. A
origem de pose permanece sempre no lado azul, como exigido pelo MegaTag2; o PathPlanner faz apenas
o espelhamento da trajetória quando a aliança é vermelha.

As transforms atuais são provisórias. Meça cada câmera a partir do centro geométrico do robô usando
a convenção WPILib: `+X` para frente, `+Y` para a esquerda, `+Z` para cima e yaw positivo
anti-horário. Um erro de poucos centímetros ou graus aqui aparece diretamente como erro sistemático
de pose.

Antes do robô real:

1. Configure a mesma família/mapa de AprilTags 2026 nas três Limelights.
2. Confirme os nomes de NetworkTables e o IP de cada câmera.
3. Meça as transforms e altere `ConfigVision`.
4. Verifique exposição, ganho, foco e latência individualmente.
5. Compare a pose de cada câmera parada em vários pontos do campo antes de habilitar fusão em auto.

### Filtro de confiabilidade

Um frame é rejeitado se tiver tag desconhecida, timestamp velho/futuro, pose fora do campo,
altura `Z` impossível, roll/pitch impossível, distância fora do limite, ambiguidade alta, rotação
rápida ou inovação de pose excessiva. Embora a interface da Limelight às vezes seja descrita como
“altura Y”, na convenção 3D da WPILib a altura do robô é **Z**.

O heading do MegaTag2 recebe desvio-padrão muito alto. Assim, as câmeras corrigem principalmente
X/Y e o Pigeon 2 continua sendo a fonte de rotação. Os números atuais são conservadores e devem ser
ajustados somente a partir de logs do robô real.

## Simulação das três câmeras

`LimelightSimulation` calcula a pose de cada câmera, testa distância, FOV e lado visível da tag,
adiciona ruído determinístico e publica as mesmas chaves NT usadas pelo hardware. Ela não simula
pixels, iluminação, motion blur nem oclusão por peças/outros robôs.

No AdvantageScope/Elastic, observe:

- `/VisionSim/limelight-front/VisibleTagIds`
- `/VisionSim/limelight-left/VisibleTagIds`
- `/VisionSim/limelight-right/VisibleTagIds`
- `/Vision/<camera>/Accepted`, `Reason`, `Confidence` e `AverageDistanceMeters`

Isso permite testar qual câmera deveria ver cada tag e por que um frame foi aceito ou rejeitado.

## Colisão, histórico e recuperação

O Pigeon 2 mede aceleração linear e velocidade angular; ele **não mede velocidade translacional**.
Em velocidade constante de 2,7 m/s é normal o acelerômetro medir aproximadamente zero. Portanto,
“comando 2,7 m/s + aceleração zero” não prova que o robô está bloqueado.

O monitor combina impacto/jerk, velocidade pedida, velocidade estimada pelas rodas e divergência
entre aceleração das rodas e IMU. Mesmo assim, uma roda girando contra uma parede pode enganar a
odometria. Quando há bloqueio confirmado, o solver aplica X-lock. Após o piloto soltar a translação
e o IMU estabilizar, o estado volta por `RECOVERING` até `NOMINAL`.

O buffer salva pose e velocidades a cada 20 ms. Ao detectar impacto, uma pose anterior é reconstruída
integrando velocidades robot-relative até antes da amostra suspeita. O rollback automático está
desligado por padrão, pois outro robô pode realmente deslocar o chassi e voltar a uma pose antiga
pioraria o erro. O botão **Back** aplica manualmente a pose recomendada somente se ela estiver dentro
do limite configurado.

Durante um bloqueio, a saída do PathPlanner fica em X-lock, mas o relógio interno da trajetória
continua avançando. Bloqueios longos devem cancelar/selecionar uma rotina de recuperação futura em
vez de tentar alcançar agressivamente o ponto já avançado.

## PathPlanner

O `AutoBuilder` usa:

- pose e velocidades robot-relative do estimador Phoenix;
- reset de pose com origem azul;
- `PPHolonomicDriveController`;
- feedforwards X/Y de força do PathPlanner enviados a `ApplyRobotSpeeds`;
- espelhamento para a aliança vermelha;
- interlock do monitor de colisão.

Abra a pasta `Swerve` no PathPlanner e crie arquivos em `src/main/deploy/pathplanner`. Os autos
aparecem automaticamente no chooser `Autonomo`.

Os valores em `settings.json` são exemplos. Antes de confiar em uma trajetória, meça massa, MOI,
posição dos módulos, raio efetivo da roda, corrente, coeficiente de atrito e velocidade máxima. Um
modelo físico errado pode fazer os feedforwards piorarem a trajetória; nesse caso, valide primeiro
sem feedforward e corrija as unidades/medições.

## CPU, GC e CAN

Não existe aqui uma leitura portátil e confiável de “CPU %” da roboRIO. Em vez disso, o monitor
publica proxies úteis:

- maior intervalo recente do loop principal;
- temperatura da CPU;
- memória usada pela JVM;
- utilização, bus-off, TX-full, REC e TEC do barramento CAN.

A telemetria pesada do swerve foi reduzida para 20 Hz, visão roda a 30 Hz e dashboards de visão a
10 Hz. As Limelights fazem o processamento de imagem, deixando a roboRIO apenas validar e fundir
poses. O código também reutiliza requests Phoenix e sinais Pigeon, evitando alocações no loop de
odometria.

Use os alertas como sintomas, não como diagnóstico final. Mantenha CAN abaixo de aproximadamente
85% em operação normal, investigue loops acima de 30 ms e confirme tudo com logs de uma partida.

## Checklist de precisão no hardware

1. Gere novamente o swerve no Phoenix Tuner X 2026.
2. Calibre o zero absoluto de cada CANcoder com rodas mecanicamente alinhadas.
3. Confirme direção, inversão, magnet health e ausência de saltos no sinal absoluto.
4. Use `FusedCANcoder`, mas confira a relação steer/encoder e o mecanismo sem folga excessiva.
5. Meça o raio **efetivo** das rodas pelo deslocamento real, não apenas pelo diâmetro nominal.
6. Faça SysId de drive, steer e rotação com bateria carregada e massa final do robô.
7. Calibre montagem, orientação e bias do Pigeon 2; monte-o rigidamente longe de vibração excessiva.
8. Verifique corrente de slip, corrente de alimentação e queda de tensão sob carga.
9. Use CANivore para o swerve quando disponível e ajuste frequências de status sem passar de 85%.
10. Faça testes repetidos de ida/volta e rotação; se o erro cresce com distância, revise raio/redução;
    se cresce com giro, revise posições dos módulos, coupling e gyro.
11. Só depois ajuste covariâncias de visão, gates e ganhos do PathPlanner.

Encoders extras raramente corrigem uma geometria/calibração ruim. Antes de adicionar sensores,
registre erros de módulo, CANcoder, Pigeon e visão para identificar qual grandeza realmente deriva.

## Executar

No terminal WPILib, dentro da pasta `Swerve`:

```powershell
.\gradlew.bat build
.\gradlew.bat simulateJava
```

Selecione `Autonomous` no Sim Driver Station, escolha a rotina em `Autonomo` e habilite. No
AdvantageScope, adicione `Swerve/Pose`, `Swerve/ModuleStates`, `Swerve/ModuleTargets` e os tópicos de
visão descritos acima.

## Controles

- Analógico esquerdo: translação.
- Analógico direito X: rotação.
- A: freio em X.
- B: apontar módulos na direção do analógico esquerdo.
- LB: redefinir a frente field-centric.
- Back: aplicar manualmente uma pose histórica recomendada.
- RT: `COLLECTING`; ao soltar, `HOLDING_GAME_PIECE`.
- LT: `SCORING` com X-lock; ao soltar, `IDLE`.
- Y: `CLIMBING`.
- X: `IDLE`.
- RB: simular posse de peça.

## Fontes estudadas

- [Liposor/Odometry](https://github.com/Liposor/Odometry) — ideias de filtragem mecânica; o
  repositório contém descrição, mas não uma implementação verificável.
- [Limelight MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
  e [API NetworkTables](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api).
- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
  e [AprilTags](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/index.html).
- [CTRE Phoenix 6 Swerve](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html)
  e [Status Signals](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/status-signals.html).
- [PathPlanner AutoBuilder](https://pathplanner.dev/pplib-build-an-auto.html) e
  [Robot Config](https://pathplanner.dev/robot-config.html).
- [Citrus Circuits 2026](https://github.com/frc1678/C2026-Public),
  [Citrus Circuits 2025](https://github.com/frc1678/C2025-Public),
  [Team 254 2025](https://github.com/Team254/FRC-2025-Public),
  [Team 2910 2025](https://github.com/FRCTeam2910/2025CompetitionRobot-Public) e
  [MapleSim](https://github.com/Team254/maple-sim).

Os repositórios de equipes foram usados como referência de arquitetura, não como prova de que os
mesmos ganhos, filtros ou modelo físico funcionarão neste robô.
