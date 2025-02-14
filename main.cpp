#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <climits>
#include <algorithm>
#include "TADgrafo.h"

// Funcion BFS que devuelve el recorrido en una lista de vertices
ListaVertice BFS(Grafo g, int start) {
    g = desmarcarGrafo(g);
    std::queue<int> cola;
    std::vector<int> recorrido; // Usar vector para mantener el orden

    g = marcarVertice(g, start);
    cola.push(start);

    while (!cola.empty()) {
        int actual = cola.front();
        cola.pop();

        recorrido.push_back(actual); // Agregar al vector

        ListaVertice sucesoresLista = sucesores(g, actual);
        while (sucesoresLista != nullptr) {
            if (!marcadoVertice(g, sucesoresLista->dato)) {
                g = marcarVertice(g, sucesoresLista->dato);
                cola.push(sucesoresLista->dato);
            }
            sucesoresLista = sucesoresLista->sig;
        }
    }

    // Convertir vector a lista enlazada ordenada
    ListaVertice listaRecorrido = nullptr;
    for (int vertice : recorrido) {
        ListaVertice nuevo = (ListaVertice)malloc(sizeof(struct NodoV));
        nuevo->dato = vertice;
        nuevo->marcado = 0;
        nuevo->sig = listaRecorrido;
        listaRecorrido = nuevo;
    }

    // Invertir la lista enlazada para que quede en el orden del recorrido BFS
    ListaVertice listaOrdenada = nullptr;
    while (listaRecorrido != nullptr) {
        ListaVertice temp = listaRecorrido;
        listaRecorrido = listaRecorrido->sig;

        temp->sig = listaOrdenada;
        listaOrdenada = temp;
    }

    return listaOrdenada;
}

// Funcion DFS que devuelve el recorrido en una lista de vértices
ListaVertice DFS(Grafo g, int start) {
    g = desmarcarGrafo(g);
    std::stack<int> pila;
    ListaVertice recorrido = nullptr;

    g = marcarVertice(g, start);
    pila.push(start);

    while (!pila.empty()) {
        int actual = pila.top();
        pila.pop();

        ListaVertice nuevo = (ListaVertice)malloc(sizeof(struct NodoV));
        nuevo->dato = actual;
        nuevo->marcado = 0;
        nuevo->sig = recorrido;
        recorrido = nuevo;

        ListaVertice sucesoresLista = sucesores(g, actual);
        while (sucesoresLista != nullptr) {
            if (!marcadoVertice(g, sucesoresLista->dato)) {
                g = marcarVertice(g, sucesoresLista->dato);
                pila.push(sucesoresLista->dato);
            }
            sucesoresLista = sucesoresLista->sig;
        }
    }

    return recorrido;
}

// Funcion Dijkstra para calcular el camino mínimo
std::vector<int> Dijkstra(Grafo g, int start, int end) {
    int numVertices = cantidadVertices(g);
    std::vector<int> distancia(numVertices + 1, INT_MAX);
    std::vector<int> predecesor(numVertices + 1, -1);
    std::vector<bool> visitado(numVertices + 1, false);

    distancia[start] = 0;

    for (int i = 1; i <= numVertices; ++i) {
        // Encuentra el vértice con la menor distancia que no ha sido visitado
        int minDist = INT_MAX, u = -1;
        for (int j = 1; j <= numVertices; ++j) {
            if (!visitado[j] && distancia[j] < minDist) {
                minDist = distancia[j];
                u = j;
            }
        }

        if (u == -1) break; // Todos los vértices restantes son inaccesibles
        visitado[u] = true;

        // Actualiza las distancias de los sucesores
        ListaVertice sucesoresLista = sucesores(g, u);
        while (sucesoresLista != nullptr) {
            int v = sucesoresLista->dato;
            int peso = pesoArco(g, u, v);
            if (!visitado[v] && distancia[u] + peso < distancia[v]) {
                distancia[v] = distancia[u] + peso;
                predecesor[v] = u;
            }
            sucesoresLista = sucesoresLista->sig;
        }
    }

    // Construir el camino más corto desde start hasta end
    std::vector<int> camino;
    for (int at = end; at != -1; at = predecesor[at]) {
        camino.push_back(at);
    }
    std::reverse(camino.begin(), camino.end());

    // Verificar si el destino es accesible
    if (camino.size() == 1 && camino[0] != start) {
        camino.clear(); // El destino no es accesible desde el inicio
    }

    return camino;
}

// Funcion para imprimir un recorrido
void imprimirRecorrido(ListaVertice recorrido) {
    while (recorrido != nullptr) {
        std::cout << recorrido->dato << " ";
        recorrido = recorrido->sig;
    }
    std::cout << std::endl;
}

// Funcion para imprimir distancias de Dijkstra
void imprimirDistancias(const std::vector<int>& distancias) {
    for (size_t i = 1; i < distancias.size(); ++i) {
        if (distancias[i] == INT_MAX) {
            std::cout << "Distancia al vertice " << i << ": INFINITO" << std::endl;
        } else {
            std::cout << "Distancia al vertice " << i << ": " << distancias[i] << std::endl;
        }
    }
}

// Funcion Bellman-Ford para calcular el camino minimo
std::pair<std::vector<int>, std::vector<int>> BellmanFord(Grafo g, int start, int end) {
    int numVertices = cantidadVertices(g);
    std::vector<int> distancia(numVertices + 1, INT_MAX);
    std::vector<int> predecesor(numVertices + 1, -1); // Para reconstruir el camino

    distancia[start] = 0;

    // Relajar los arcos numVertices - 1 veces
    for (int i = 1; i <= numVertices - 1; ++i) {
        ListaArco actual = g.a;
        while (actual != nullptr) {
            int u = actual->origen;
            int v = actual->destino;
            int peso = actual->costo;

            if (distancia[u] != INT_MAX && distancia[u] + peso < distancia[v]) {
                distancia[v] = distancia[u] + peso;
                predecesor[v] = u; // Registrar de dónde viene
            }
            actual = actual->sig;
        }
    }

    // Detectar ciclos negativos
    ListaArco actual = g.a;
    while (actual != nullptr) {
        int u = actual->origen;
        int v = actual->destino;
        int peso = actual->costo;

        if (distancia[u] != INT_MAX && distancia[u] + peso < distancia[v]) {
            std::cerr << "El grafo contiene un ciclo negativo." << std::endl;
            return {{}, {}}; // Retorna vectores vacíos en caso de ciclo negativo
        }
        actual = actual->sig;
    }

    // Construir el camino hacia el destino
    std::vector<int> camino;
    if (distancia[end] != INT_MAX) {
        for (int v = end; v != -1; v = predecesor[v]) {
            camino.push_back(v);
        }
        std::reverse(camino.begin(), camino.end());
    }

    return {distancia, camino};
}

// Funcion Prim para calcular el arbol de expansion minima (MST)
std::vector<std::pair<int, int>> Prim(Grafo g, int start) {
    int numVertices = cantidadVertices(g);
    std::vector<int> key(numVertices + 1, INT_MAX); // Valores para escoger el menor peso
    std::vector<int> parent(numVertices + 1, -1);  // Para guardar el arbol
    std::vector<bool> inMST(numVertices + 1, false); // Para saber si el vertice ya esta en el MST

    key[start] = 0; // La clave del vértice inicial es 0

    for (int count = 1; count <= numVertices; ++count) {
        // Encuentra el vértice con la clave más pequeña que no está en el MST
        int minKey = INT_MAX, u = -1;
        for (int v = 1; v <= numVertices; ++v) {
            if (!inMST[v] && key[v] < minKey) {
                minKey = key[v];
                u = v;
            }
        }

        if (u == -1) break; // Si no hay vértices conectados restantes

        inMST[u] = true; // Agregar el vértice al MST

        // Actualiza las claves y padres de los sucesores
        ListaVertice sucesoresLista = sucesores(g, u);
        while (sucesoresLista != nullptr) {
            int v = sucesoresLista->dato;
            int peso = pesoArco(g, u, v);

            // Si el vértice no está en el MST y el peso del arco es menor que la clave actual
            if (!inMST[v] && peso < key[v]) {
                key[v] = peso;
                parent[v] = u;
            }

            sucesoresLista = sucesoresLista->sig;
        }
    }

    // Construir el MST como una lista de pares (u, v)
    std::vector<std::pair<int, int>> mst;
    for (int i = 1; i <= numVertices; ++i) {
        if (parent[i] != -1) {
            mst.push_back({parent[i], i});
        }
    }

    return mst;
}

struct Arco {
  int origen, destino, peso;
};

// Estructura para el conjunto disjunto
struct Conjunto {
    std::vector<int> padre;
    std::vector<int> rango;

    Conjunto(int n) {
        padre.resize(n + 1);
        rango.resize(n + 1, 0);
        for (int i = 1; i <= n; ++i) {
            padre[i] = i; // Cada vertice es su propio padre al principio
        }
    }

    // Encuentra el representante de un conjunto
    int encontrar(int x) {
        if (padre[x] != x) {
            padre[x] = encontrar(padre[x]); // Compresion de caminos
        }
        return padre[x];
    }

    // Une dos conjuntos
    void unir(int x, int y) {
        int raizX = encontrar(x);
        int raizY = encontrar(y);

        if (raizX != raizY) {
            // Union por rango
            if (rango[raizX] > rango[raizY]) {
                padre[raizY] = raizX;
            } else if (rango[raizX] < rango[raizY]) {
                padre[raizX] = raizY;
            } else {
                padre[raizY] = raizX;
                rango[raizX]++;
            }
        }
    }
};

std::vector<std::pair<int, int>> Kruskal(Grafo g) {
    int numVertices = cantidadVertices(g);
    std::vector<Arco> arcos;

    // Recopilar todos los arcos del grafo
    ListaArco listaArcos = g.a;
    while (listaArcos != nullptr) {
        arcos.push_back({listaArcos->origen, listaArcos->destino, listaArcos->costo});
        listaArcos = listaArcos->sig;
    }

    // Ordenar los arcos por peso
    std::sort(arcos.begin(), arcos.end(), [](const Arco& a, const Arco& b) {
        return a.peso < b.peso;
    });

    Conjunto conjunto(numVertices);
    std::vector<std::pair<int, int>> mst;

    // Iterar sobre los arcos y agregar al MST si no forman un ciclo
    for (const auto& arco : arcos) {
        int u = arco.origen;
        int v = arco.destino;

        // Si los vertices u y v no están en el mismo conjunto, unirlos
        if (conjunto.encontrar(u) != conjunto.encontrar(v)) {
            conjunto.unir(u, v);
            mst.push_back({u, v});
        }
    }

    return mst;
}

// Funcion para imprimir el arbol de expansion minima
void imprimirMST(const std::vector<std::pair<int, int>>& mst) {
    std::cout << "Arbol de expansion minima (Prim):" << std::endl;
    for (const auto& arco : mst) {
        std::cout << arco.first << " - " << arco.second << std::endl;
    }
}

void imprimirMSTKruskal(const std::vector<std::pair<int, int>>& mst) {
    std::cout << "Arbol de expansion minima (Kruskal):" << std::endl;
    for (const auto& arco : mst) {
        std::cout << arco.first << " - " << arco.second << std::endl;
    }
}


// Funcion para crear un grafo de ejemplo
Grafo crearGrafoDeEjemplo() {
    Grafo g = crearGrafo();
    // Insertando vertices (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
    for (int i = 1; i <= 10; i++) {
        g = insertarVertice(g, i);
    }

    // Insertando arcos con sus respectivos costos

    // Arcos desde (1)
    g = insertarArco(g, 1, 2, 3);   // 1 -> 2, costo 3
    g = insertarArco(g, 1, 3, 2);   // 1 -> 3, costo 2
    g = insertarArco(g, 1, 4, 4);   // 1 -> 4, costo 4

    // Arcos desde (2)
    g = insertarArco(g, 2, 1, 3);   // 2 -> 1, costo 3
    g = insertarArco(g, 2, 5, 2);   // 2 -> 5, costo 2
    
    // Arcos desde (3)
    g = insertarArco(g, 3, 1, 2);   // 3 -> 1, costo 2
    g = insertarArco(g, 3, 5, 3);   // 3 -> 5, costo 3
    g = insertarArco(g, 3, 6, 5);   // 3 -> 6, costo 5
    g = insertarArco(g, 3, 7, 4);   // 3 -> 7, costo 4

    // Arcos desde (4)
    g = insertarArco(g, 4, 1, 4);   // 4 -> 1, costo 4
    g = insertarArco(g, 4, 7, 3);   // 4 -> 7, costo 3

    // Arcos desde (5)
    g = insertarArco(g, 5, 2, 2);   // 5 -> 2, costo 2
    g = insertarArco(g, 5, 3, 3);   // 5 -> 3, costo 3
    g = insertarArco(g, 5, 6, 1);   // 5 -> 6, costo 1
    g = insertarArco(g, 5, 8, 3);   // 5 -> 8, costo 3

    // Arcos desde (6)
    g = insertarArco(g, 6, 3, 5);   // 6 -> 3, costo 5
    g = insertarArco(g, 6, 5, 1);   // 6 -> 5, costo 1
    g = insertarArco(g, 6, 7, 1);   // 6 -> 7, costo 1
    g = insertarArco(g, 6, 10, 4);   // 6 -> 10, costo 4

    // Arcos desde (7)
    g = insertarArco(g, 7, 3, 4);   // 7 -> 3, costo 4
    g = insertarArco(g, 7, 4, 3);   // 7 -> 4, costo 3
    g = insertarArco(g, 7, 6, 1);   // 7 -> 6, costo 1
    g = insertarArco(g, 7, 9, 2);   // 7 -> 9, costo 2

    // Arcos desde (8)
    g = insertarArco(g, 8, 5, 3);   // 8 -> 5, costo 3
    g = insertarArco(g, 8, 9, 1);   // 8 -> 9, costo 1 
    g = insertarArco(g, 8, 10, 2);   // 8 -> 10, costo 2

    // Arcos desde (9)
    g = insertarArco(g, 9, 7, 2);   // 9 -> 7, costo 2
    g = insertarArco(g, 9, 8, 1);   // 9 -> 8, costo 1
    g = insertarArco(g, 9, 10, 2);   // 9 -> 10, costo 2

    // Arcos desde (10)
    g = insertarArco(g, 10, 6, 4);   // 10 -> 6, costo 4
    g = insertarArco(g, 10, 8, 2);   // 10 -> 8, costo 2
    g = insertarArco(g, 10, 9, 2);   // 10 -> 9, costo 2

    return g;
}

int main() {
    // Crear un grafo de ejemplo
    Grafo g = crearGrafoDeEjemplo();

    // Mostrar lista de vertices y arcos
    std::cout << "Lista de vertices:" << std::endl;
    imprimirListaV(g);
    std::cout << "\n\nLista de arcos:" << std::endl;
    imprimirListaA(g);

    // Menú para seleccionar algoritmo
    int opcion, inicio;

    do {
        std::cout << "\n\nSeleccione el algoritmo:" << std::endl;
        std::cout << "1. BFS" << std::endl;
        std::cout << "2. DFS" << std::endl;
        std::cout << "3. Dijkstra" << std::endl;
        std::cout << "4. Bellman-Ford" << std::endl;
        std::cout << "5. Prim" << std::endl;
        std::cout << "6. Kruskal" << std::endl;
        std::cout << "7. Salir" << std::endl;
        std::cout << "Opcion: ";
        std::cin >> opcion;

        if (opcion == 7) {
            std::cout << "Saliendo..." << std::endl;
            return 0;
        }

        if (opcion != 5) {
            std::cout << "Ingrese el vertice inicial: ";
            std::cin >> inicio;
        }

        switch (opcion) {
            case 1: {
                ListaVertice recorrido = BFS(g, inicio);
                std::cout << "\nRecorrido BFS: ";
                imprimirRecorrido(recorrido);
                break;
            }
            case 2: {
                ListaVertice recorrido = DFS(g, inicio);
                std::cout << "\nRecorrido DFS: ";
                imprimirRecorrido(recorrido);
                break;
            }
            case 3: {
                int destino;
                std::cout << "Ingrese el vertice destino: ";
                std::cin >> destino;

                std::vector<int> camino = Dijkstra(g, inicio, destino);
                
                if (!camino.empty()) {
                    std::cout << "\nCamino mas corto desde " << inicio << " hasta " << destino << ": ";
                    for (int vertice : camino) {
                        std::cout << vertice << " ";
                    }
                    std::cout << std::endl;
                } else {
                    std::cout << "\nNo hay camino desde " << inicio << " hasta " << destino << "." << std::endl;
                }
                break;
            }
            case 4: {
                int destino;
                std::cout << "Ingrese el vertice destino: ";
                std::cin >> destino;

                auto resultado = BellmanFord(g, inicio, destino);
                const auto& distancias = resultado.first;
                const auto& camino = resultado.second;

                if (distancias.empty()) {
                    std::cout << "\nNo se pudo calcular debido a un ciclo negativo." << std::endl;
                } else {
                    std::cout << "\nDistancias desde el vertice " << inicio << ":" << std::endl;
                    imprimirDistancias(distancias);

                    if (!camino.empty()) {
                        std::cout << "\nCamino mas corto desde " << inicio << " hasta " << destino << ": ";
                        for (int vertice : camino) {
                            std::cout << vertice << " ";
                        }
                        std::cout << std::endl;
                    } else {
                        std::cout << "\nNo hay camino hacia el vertice " << destino << "." << std::endl;
                    }
                }
                break;
            }
            case 5: {
                std::vector<std::pair<int, int>> mst = Prim(g, 1); // Suponemos que empieza desde el vertice 1
                imprimirMST(mst);
                break;
            }
            case 6: {
                std::vector<std::pair<int, int>> mst = Kruskal(g); // Ejecutar Kruskal
                imprimirMSTKruskal(mst); // Imprimir el MST
                break;
            }
            default:
                std::cout << "Opcion no valida." << std::endl;
        }
    } while (opcion != 7);

    return 0;
}
