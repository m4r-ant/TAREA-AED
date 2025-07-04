# Tarea de Grafos - Path Finder para Lima

## Integrantes: 
- 1 ____   _____
- 2 ____   _____
- 3 ____   _____

## Objetivo: 
El objetivo de esta tarea es implementar un **Path Finder** para la ciudad de Lima. 

<p align="center">
    <img src=https://github.com/utec-cs-aed/homework_graph/assets/79115974/b63f69db-17eb-417a-8aa1-8483d8dcdaf0 / >
</p>

## Descripción del Proyecto

Este proyecto implementa un sistema de búsqueda de rutas en un grafo que representa la red vial de Lima. El sistema permite visualizar en tiempo real cómo diferentes algoritmos de búsqueda encuentran el camino óptimo entre dos puntos seleccionados por el usuario.

### Características Principales:
- **Visualización Interactiva**: Interfaz gráfica que permite seleccionar puntos de origen y destino
- **Múltiples Algoritmos**: Implementación de Dijkstra, Best First Search y A*
- **Animación en Tiempo Real**: Visualización paso a paso de la ejecución de los algoritmos
- **Dataset Real**: Utiliza datos reales de la red vial de Lima

## Dependencias

Para esta tarea se solicita utilizar ```C++17``` y la librería ```SFML 2.5```

- Para instalar ```SFML 2.5```:

    - [Windows](https://www.youtube.com/watch?v=HkPRG0vfObc)
    - [MacOS y Linux](https://www.youtube.com/playlist?list=PLvv0ScY6vfd95GMoMe2zc4ZgGxWYj3vua)

Cuando se instale la librería, probar que las siguientes líneas del ```CMakeLists.txt``` encuentren la librería adecuadamente.
```cmake
find_package(SFML 2.5 COMPONENTS graphics window REQUIRED)
if(SFML_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-graphics sfml-window)
else()
    message("SFML not found")
endif()
```

## Compilación y Ejecución

### Prerrequisitos:
- CMake 3.27 o superior
- Compilador C++ compatible con C++17
- SFML 2.5 instalado

### Pasos para compilar:

1. **Crear directorio de build:**
   ```bash
   mkdir build
   cd build
   ```

2. **Configurar con CMake:**
   ```bash
   cmake ..
   ```

3. **Compilar:**
   ```bash
   cmake --build .
   ```

4. **Ejecutar:**
   ```bash
   ./homework_graph
   ```

## Uso del Programa

### Controles:
- **Click Izquierdo**: Seleccionar nodo de origen (verde) y destino (cyan)
- **D**: Ejecutar algoritmo de Dijkstra
- **A**: Ejecutar algoritmo A*
- **B**: Ejecutar algoritmo Best First Search
- **R**: Resetear la simulación actual
- **E**: Mostrar/ocultar todas las aristas visitadas
- **Q**: Salir del programa

### Flujo de Uso:
1. Ejecutar el programa
2. Hacer click en el punto de origen (aparecerá en verde)
3. Hacer click en el punto de destino (aparecerá en cyan)
4. Presionar D, A o B para ejecutar el algoritmo correspondiente
5. Observar la animación del algoritmo en tiempo real
6. Presionar R para limpiar y seleccionar nuevos puntos

## Dataset
El dataset consiste de dos csv:

- *nodes.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/6a68cf06-196a-4605-83a7-3183e9a3f0ec)

    Contiene información de los nodos (intersecciones) de la red vial:
    - `id`: Identificador único del nodo
    - `latitude`: Coordenada Y del nodo
    - `longitude`: Coordenada X del nodo

- *edges.csv*

    ![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/247bbbd7-6203-45f4-8196-fcb0434b0f1d)

    Contiene información de las aristas (calles) de la red vial:
    - `src_id`: ID del nodo origen
    - `dest_id`: ID del nodo destino
    - `max_speed`: Velocidad máxima permitida (km/h)
    - `length`: Longitud de la calle (metros)
    - `oneway`: Si es calle de un solo sentido
    - `lanes`: Número de carriles

## Algoritmos a Implementar

Se les solicita implementar tres algoritmos para búsqueda en grafos:

### 1. Dijkstra
- **Propósito**: Encuentra el camino más corto desde un nodo origen a todos los demás nodos
- **Características**: 
  - Garantiza el camino óptimo
  - Utiliza una cola de prioridad
  - Complejidad: O((V + E) log V) donde V = vértices, E = aristas

### 2. Best First Search
- **Propósito**: Encuentra un camino usando heurísticas para guiar la búsqueda
- **Características**:
  - No garantiza el camino óptimo
  - Utiliza heurística de distancia euclidiana
  - Complejidad: O(V log V) en el peor caso

### 3. A*
- **Propósito**: Combina Dijkstra con heurísticas para encontrar el camino óptimo de manera eficiente
- **Características**:
  - Garantiza el camino óptimo si la heurística es admisible
  - Utiliza f(n) = g(n) + h(n) donde g(n) es el costo real y h(n) es la heurística
  - Complejidad: O(V log V) en el peor caso

### Heurística Sugerida:
- **Distancia Euclidiana**: Calcular la distancia en línea recta entre dos puntos
- **Fórmula**: h(n) = √((x₂-x₁)² + (y₂-y₁)²)

## Análisis de Complejidad Computacional

### Dijkstra:
- **Complejidad Temporal**: O((V + E) log V)
  - V iteraciones para procesar todos los nodos
  - E iteraciones para revisar todas las aristas
  - log V por operaciones de cola de prioridad
- **Complejidad Espacial**: O(V)
  - Array de distancias: O(V)
  - Cola de prioridad: O(V)
  - Array de padres: O(V)

### Best First Search:
- **Complejidad Temporal**: O(V log V)
  - V iteraciones para procesar todos los nodos
  - log V por operaciones de cola de prioridad
- **Complejidad Espacial**: O(V)
  - Cola de prioridad: O(V)
  - Set de nodos visitados: O(V)

### A*:
- **Complejidad Temporal**: O(V log V)
  - Similar a Dijkstra pero con mejor rendimiento en la práctica
  - La heurística reduce el número de nodos explorados
- **Complejidad Espacial**: O(V)
  - Misma que Dijkstra

## Estructura del Código

### Archivos Principales:
- `main.cpp`: Punto de entrada del programa
- `gui.h`: Interfaz gráfica y manejo de eventos
- `graph.h`: Estructura del grafo y operaciones básicas
- `node.h`: Definición de nodos (vértices)
- `edge.h`: Definición de aristas
- `path_finding_manager.h`: Implementación de algoritmos de búsqueda
- `window_manager.h`: Manejo de la ventana gráfica

### Clases Principales:
- **Node**: Representa un vértice del grafo con coordenadas y aristas
- **Edge**: Representa una arista con información de velocidad, longitud, etc.
- **Graph**: Contiene todos los nodos y aristas, maneja el parsing de CSV
- **PathFindingManager**: Implementa los algoritmos de búsqueda
- **GUI**: Maneja la interfaz de usuario y eventos
- **WindowManager**: Wrapper para la ventana SFML

## Diagrama de clases UML 

![image](https://github.com/utec-cs-aed/homework_graph/assets/79115974/f5a3d89e-cb48-4715-b172-a17e6e27ee24)

## Tareas Pendientes

### Implementación de Algoritmos:
1. **Dijkstra** en `path_finding_manager.h`:
   - Implementar cola de prioridad
   - Mantener array de distancias
   - Actualizar padres para reconstruir el camino

2. **Best First Search** en `path_finding_manager.h`:
   - Implementar cola de prioridad basada en heurística
   - Usar distancia euclidiana como heurística
   - No requiere array de distancias

3. **A*** en `path_finding_manager.h`:
   - Combinar costo real (g) con heurística (h)
   - f(n) = g(n) + h(n)
   - Mantener array de costos reales

### Funciones Auxiliares:
- **set_final_path()**: Reconstruir el camino desde el mapa de padres
- **render()**: Visualizar el progreso del algoritmo
- **exec()**: Coordinar la ejecución de los algoritmos

## Entregables

1. **Código Implementado**: Todos los algoritmos funcionando correctamente
2. **Análisis de Complejidad**: Documentación de la complejidad de cada algoritmo
3. **Video Demostrativo**: Video de 2 minutos mostrando la funcionalidad visual de cada algoritmo
4. **README Completo**: Documentación del proyecto (este archivo)

## Notas Importantes

- **Heurística**: Usar distancia euclidiana como heurística para A* y Best First Search
- **Visualización**: El programa debe mostrar el progreso del algoritmo en tiempo real
- **Optimización**: A* debe ser más eficiente que Dijkstra en la práctica
- **Robustez**: Manejar casos edge como nodos no conectados

## Recursos Adicionales

- [Documentación SFML](https://www.sfml-dev.org/documentation.php)
- [Algoritmo de Dijkstra](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)
- [Algoritmo A*](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Best First Search](https://en.wikipedia.org/wiki/Best-first_search)