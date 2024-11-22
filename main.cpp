#include <iostream>
#include <queue>
#include <stack>
#include <vector>
#include <limits.h>
#include <algorithm>
#include "TADgrafo.h"

// Función BFS que devuelve el recorrido en una lista de vértices
ListaVertice BFS(Grafo g, int start) {
    g = desmarcarGrafo(g);
    std::queue<int> cola;
    ListaVertice recorrido = nullptr;

    g = marcarVertice(g, start);
    cola.push(start);

    while (!cola.empty()) {
        int actual = cola.front();
        cola.pop();

        ListaVertice nuevo = (ListaVertice)malloc(sizeof(struct NodoV));
        nuevo->dato = actual;
        nuevo->marcado = 0;
        nuevo->sig = recorrido;
        recorrido = nuevo;

        ListaVertice sucesoresLista = sucesores(g, actual);
        while (sucesoresLista != nullptr) {
            if (!marcadoVertice(g, sucesoresLista->dato)) {
                g = marcarVertice(g, sucesoresLista->dato);
                cola.push(sucesoresLista->dato);
            }
            sucesoresLista = sucesoresLista->sig;
        }
    }

    return recorrido;
}

// Función DFS que devuelve el recorrido en una lista de vértices
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

// Función Dijkstra para calcular el camino mínimo
std::vector<int> Dijkstra(Grafo g, int start) {
    int numVertices = cantidadVertices(g);
    std::vector<int> distancia(numVertices + 1, INT_MAX);
    std::vector<bool> visitado(numVertices + 1, false);

    distancia[start] = 0;
    for (int i = 1; i <= numVertices; ++i) {
        // Encuentra el vértice con menor distancia que no ha sido visitado
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
            }
            sucesoresLista = sucesoresLista->sig;
        }
    }

    return distancia;
}

// Función para imprimir un recorrido
void imprimirRecorrido(ListaVertice recorrido) {
    while (recorrido != nullptr) {
        std::cout << recorrido->dato << " ";
        recorrido = recorrido->sig;
    }
    std::cout << std::endl;
}

// Función para imprimir distancias de Dijkstra
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
std::vector<int> BellmanFord(Grafo g, int start) {
    int numVertices = cantidadVertices(g);
    std::vector<int> distancia(numVertices + 1, INT_MAX);
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
            return {}; // Retorna un vector vacio en caso de ciclo negativo
        }
        actual = actual->sig;
    }

    return distancia;
}

// Funcion Prim para calcular el arbol de expansion minima (MST)
std::vector<std::pair<int, int>> Prim(Grafo g, int start) {
    int numVertices = cantidadVertices(g);
    std::vector<int> key(numVertices + 1, INT_MAX); // Valores para escoger el menor peso
    std::vector<int> parent(numVertices + 1, -1);  // Para guardar el arbol
    std::vector<bool> inMST(numVertices + 1, false); // Para saber si el vertice ya esta en el MST

    key[start] = 0;

    for (int i = 1; i <= numVertices; ++i) {
        // Encuentra el vertice con la menor clave que no esta en el MST
        int minKey = INT_MAX, u = -1;
        for (int j = 1; j <= numVertices; ++j) {
            if (!inMST[j] && key[j] < minKey) {
                minKey = key[j];
                u = j;
            }
        }

        if (u == -1) break; // No quedan vertices conectados

        inMST[u] = true;

        // Actualiza los valores de clave y padre de los vertices adyacentes
        ListaVertice sucesoresLista = sucesores(g, u);
        while (sucesoresLista != nullptr) {
            int v = sucesoresLista->dato;
            int peso = pesoArco(g, u, v);

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
            padre[i] = i; // Cada vértice es su propio padre al principio
        }
    }

    // Encuentra el representante de un conjunto
    int encontrar(int x) {
        if (padre[x] != x) {
            padre[x] = encontrar(padre[x]); // Compresión de caminos
        }
        return padre[x];
    }

    // Une dos conjuntos
    void unir(int x, int y) {
        int raizX = encontrar(x);
        int raizY = encontrar(y);

        if (raizX != raizY) {
            // Unión por rango
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

        // Si los vértices u y v no están en el mismo conjunto, unirlos
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


// Función para crear un grafo de ejemplo
Grafo crearGrafoDeEjemplo() {
    Grafo g = crearGrafo();
    g = insertarVertice(g, 1);
    g = insertarVertice(g, 2);
    g = insertarVertice(g, 3);
    g = insertarVertice(g, 4);
    g = insertarVertice(g, 5);

    g = insertarArco(g, 1, 2, 2);
    g = insertarArco(g, 1, 3, 4);
    g = insertarArco(g, 2, 4, 7);
    g = insertarArco(g, 3, 5, 3);
    g = insertarArco(g, 4, 5, 1);

    return g;
}

int main() {
    // Crear un grafo de ejemplo
    Grafo g = crearGrafoDeEjemplo();

    // Mostrar lista de vértices y arcos
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
                std::cout << "Recorrido BFS: ";
                imprimirRecorrido(recorrido);
                break;
            }
            case 2: {
                ListaVertice recorrido = DFS(g, inicio);
                std::cout << "Recorrido DFS: ";
                imprimirRecorrido(recorrido);
                break;
            }
            case 3: {
                std::vector<int> distancias = Dijkstra(g, inicio);
                std::cout << "Distancias minimas desde el vertice " << inicio << ":" << std::endl;
                imprimirDistancias(distancias);
                break;
            }
            case 4: {
                std::vector<int> distancias = BellmanFord(g, inicio);
                if (!distancias.empty()) {
                    std::cout << "Distancias minimas desde el vertice " << inicio << ":" << std::endl;
                    imprimirDistancias(distancias);
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
