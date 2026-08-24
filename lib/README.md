# Bibliotecas internas

Esta pasta guarda componentes auxiliares que fazem parte do projeto, mas não representam o
comportamento principal do robô.

- `generated`: arquivos produzidos ou mantidos a partir de ferramentas de fornecedores.
- `telemetry`: publicação de dados e integração com ferramentas de análise.
- `util`: monitoramento e utilitários reutilizáveis.
- `vision`: I/O, modelos de dados, filtros e simulação das câmeras.

As classes mantêm os pacotes `frc.robot.*` e são compiladas junto com `src/main/java` pelo source
set `main` definido no `build.gradle`. Coloque aqui apenas infraestrutura reutilizável; comandos,
subsistemas e regras específicas do robô devem continuar em `src/main/java`.
