// https://robotzero.one/esp-who-recognition-with-names/

// Solução erro para:
// fd_forward.h: No such file or directory
// https://www.youtube.com/watch?v=knxe3zkd6rA

//#include <ArduinoWebsockets.h>: Essa biblioteca é necessária para a comunicação WebSocket no projeto, permitindo que a placa ESP32 se comunique com outros dispositivos por meio de WebSocket.
//#include "esp_http_server.h": A biblioteca esp_http_server.h é utilizada para criar e gerenciar um servidor HTTP na ESP32. Isso é útil para expor serviços web ou páginas HTML.
//#include "esp_timer.h": Essa biblioteca oferece funcionalidades relacionadas ao timer na ESP32, úteis para programar tarefas que dependem do tempo.
//#include "esp_camera.h": Biblioteca responsável pelo controle da câmera na ESP32, permitindo captura de imagens e vídeo.
//#include "camera_index.h": Contém informações específicas sobre a interface da câmera, como pinos utilizados e configurações.
//#include "Arduino.h": A biblioteca padrão do Arduino, que fornece funcionalidades básicas para programação no Arduino.
//#include "fd_forward.h": Essa biblioteca provavelmente está relacionada ao reconhecimento facial, mas especificamente às declarações avançadas (forward declarations) relacionadas a esse recurso.
//#include "fr_forward.h": Assim como fd_forward.h, essa biblioteca também contém declarações avançadas (forward declarations) relacionadas ao reconhecimento facial.
//#include "fr_flash.h": Biblioteca relacionada ao reconhecimento facial que lida com o armazenamento de informações, como faces identificadas, na memória flash da ESP32.

// Inclusão de bibliotecas necessárias para o projeto
#include <ArduinoWebsockets.h> // Biblioteca para comunicação WebSocket
#include "esp_http_server.h"   // Biblioteca para servidor HTTP na ESP32
#include "esp_timer.h"         // Biblioteca para funcionalidades de timer na ESP32
#include "esp_camera.h"        // Biblioteca para controle da câmera na ESP32
#include "camera_index.h"      // Biblioteca com informações sobre a interface da câmera
#include "Arduino.h"           // Biblioteca padrão do Arduino
#include "fd_forward.h"        // Biblioteca relacionada ao reconhecimento facial (forward declarations)
#include "fr_forward.h"        // Biblioteca relacionada ao reconhecimento facial (forward declarations)
#include "fr_flash.h"          // Biblioteca relacionada ao reconhecimento facial (flash storage)

// Definição de Constantes e Variáveis:
// const char* ssid = "INTELBRAS";
// const char* password = "Anaenena";

// Set your Static IP address
// IPAddress local_IP(192, 168, 15, 253);
IPAddress local_IP(10, 0, 0, 253);
// Set your Gateway IP address
// IPAddress gateway(192, 168, 15, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);
// IPAddress primaryDNS(8, 8, 8, 8); //optional
// IPAddress secondaryDNS(8, 8, 4, 4); //optional

// const char *ssid = "VIVOFIBRA-5221";
// const char *password = "kPcsBo9tdC";

const char *ssid = "INTELBRAS";
const char *password = "Anaenena";

/* #define ENROLL_CONFIRM_TIMES 5: Esta constante define o número de confirmações necessárias para concluir o processo de enroll de uma nova face.
Por exemplo, se ENROLL_CONFIRM_TIMES for 5, significa que são necessárias 5 amostras de rosto para concluir o processo de cadastro dessa face.
#define FACE_ID_SAVE_NUMBER 7: Essa constante define o número máximo de IDs de face que podem ser salvos no sistema. Se FACE_ID_SAVE_NUMBER for 7,
então o sistema pode armazenar até 7 IDs de face diferentes. Isso é útil para limitar o número de faces que podem ser reconhecidas ou gerenciadas pelo sistema de reconhecimento facial.
 */

// Definição de constantes para o projeto:
#define ENROLL_CONFIRM_TIMES 5 // Número de confirmações necessárias para concluir o processo de enroll
#define FACE_ID_SAVE_NUMBER 7  // Número máximo de IDs de face que podem ser salvos no sistema

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Utiliza o namespace websockets para simplificar o uso das classes desse namespace:
using namespace websockets;

// using namespace websockets;: Essa declaração indica que o código utilizará todas as classes, funções e outras entidades definidas no namespace websockets
// sem precisar prefixá-las com websockets::. Isso simplifica o código ao evitar repetições e tornar o código mais legível. No entanto, é importante usá-lo com
// cautela para evitar conflitos de nomes entre namespaces.

// Funções e variáveis globais
WebsocketsServer socket_server;

/* camera_fb_t * fb = NULL;: Declara um ponteiro chamado fb do tipo camera_fb_t e inicializa-o como NULL. Esse ponteiro será usado para armazenar a imagem capturada pela câmera.
long current_millis;: Declara uma variável current_millis do tipo long para armazenar o tempo atual em milissegundos.
long last_detected_millis = 0;: Declara e inicializa a variável last_detected_millis como 0. Essa variável será usada para rastrear o tempo da última detecção.
void app_facenet_main();: Declara o protótipo da função app_facenet_main, responsável por inicializar o sistema de reconhecimento facial.
void app_httpserver_init();: Declara o protótipo da função app_httpserver_init, responsável por inicializar o servidor HTTP na placa ESP32.
typedef struct {...} http_img_process_result;: Define uma nova estrutura chamada http_img_process_result, que contém três campos: image, net_boxes e face_id. Essa estrutura será usada para armazenar resultados do processamento de imagens para comunicação com o servidor HTTP.
 */
// Declaração de variáveis globais e protótipos de funções:

camera_fb_t *fb = NULL; // Ponteiro para a estrutura da imagem capturada pela câmera, inicializado como NULL

long current_millis;           // Variável para armazenar o tempo atual em milissegundos
long last_detected_millis = 0; // Variável para armazenar o último tempo em que uma detecção foi realizada, inicializada como 0

void app_facenet_main();    // Protótipo da função app_facenet_main, responsável pela inicialização do sistema de reconhecimento facial
void app_httpserver_init(); // Protótipo da função app_httpserver_init, responsável pela inicialização do servidor HTTP

/* typedef struct { ... } http_img_process_result;: Aqui, estamos definindo uma nova estrutura chamada http_img_process_result. O typedef é utilizado para criar um novo tipo de dados chamado
http_img_process_result, que representa essa estrutura.
uint8_t *image;: Este é um membro da estrutura que é um ponteiro para uint8_t, ou seja, um ponteiro para um array de bytes que representa a imagem processada.
Esse ponteiro apontará para a área de memória onde os dados da imagem estão armazenados.
box_array_t *net_boxes;: Este é outro membro da estrutura, que é um ponteiro para box_array_t. Isso sugere que net_boxes aponta para um array de caixas de detecção.
Essas caixas de detecção provavelmente contêm informações sobre objetos detectados na imagem.
dl_matrix3d_t *face_id;: O último membro da estrutura é um ponteiro para dl_matrix3d_t, que é provavelmente usado para armazenar a matriz de identificação facial.
Esta matriz pode conter informações como características faciais extraídas durante o processo de reconhecimento facial.
Essa estrutura http_img_process_result é projetada para armazenar informações importantes resultantes do processamento de imagens em um contexto de comunicação
com o servidor HTTP. Ela contém ponteiros para diferentes tipos de dados que são usados para representar a imagem processada, caixas de detecção e identificação facial,
permitindo a manipulação dessas informações de forma organizada e eficiente. */

// Definição de uma nova estrutura
typedef struct
{
  uint8_t *image;         // Ponteiro para a imagem processada
  box_array_t *net_boxes; // Ponteiro para um array de caixas de detecção
  dl_matrix3d_t *face_id; // Ponteiro para a matriz de identificação facial
} http_img_process_result;

/* Este trecho de código define uma função estática e inline chamada app_mtmn_config que é responsável por configurar os parâmetros para o processo de detecção facial.
A função retorna uma estrutura mtmn_config_t que contém todos os parâmetros configurados para o algoritmo de detecção facial.
static inline: Essas palavras-chave são usadas para indicar que a função é estática (limitada ao escopo do arquivo em que está definida) e inline
(sugerindo ao compilador que incorpore o código diretamente onde a função é chamada para melhorar a eficiência).
mtmn_config_t: Esta é uma estrutura de dados que provavelmente contém todos os parâmetros necessários para a configuração do algoritmo de detecção facial.
Os comentários explicativos são fornecidos para cada membro da estrutura mtmn_config_t, detalhando o que cada configuração representa, como tipo de detecção (FAST),
tamanho mínimo da face, limites de pontuação (score) e supressão não máxima (NMS), entre outros parâmetros específicos do algoritmo de detecção facial.
 */

// Definição de uma função estática e inline para configurar parâmetros de detecção facial
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};               // Declara uma variável mtmn_config do tipo mtmn_config_t e a inicializa com 0
  mtmn_config.type = FAST;                       // Define o tipo de detecção facial como FAST (rápido)
  mtmn_config.min_face = 80;                     // Define o tamanho mínimo da face a ser detectada como 80 pixels
  mtmn_config.pyramid = 0.707;                   // Define o valor da pirâmide de escala para a detecção facial
  mtmn_config.pyramid_times = 4;                 // Define o número de iterações para a pirâmide de escala
  mtmn_config.p_threshold.score = 0.6;           // Define o limite de pontuação (score) para a detecção de face
  mtmn_config.p_threshold.nms = 0.7;             // Define o limite de supressão não máxima (NMS) para a detecção de face
  mtmn_config.p_threshold.candidate_number = 20; // Define o número máximo de candidatos para a detecção de face
  mtmn_config.r_threshold.score = 0.7;           // Define o limite de pontuação (score) para a detecção de rosto
  mtmn_config.r_threshold.nms = 0.7;             // Define o limite de supressão não máxima (NMS) para a detecção de rosto
  mtmn_config.r_threshold.candidate_number = 10; // Define o número máximo de candidatos para a detecção de rosto
  mtmn_config.o_threshold.score = 0.7;           // Define o limite de pontuação (score) para a detecção de olhos
  mtmn_config.o_threshold.nms = 0.7;             // Define o limite de supressão não máxima (NMS) para a detecção de olhos
  mtmn_config.o_threshold.candidate_number = 1;  // Define o número máximo de candidatos para a detecção de olhos
  return mtmn_config;                            // Retorna a configuração completa para a detecção facial
}

/* Essencialmente, a linha de código está inicializando a variável mtmn_config com os parâmetros de configuração retornados pela função app_mtmn_config(),
que define os parâmetros específicos para o algoritmo de detecção facial. Isso é útil para garantir que a configuração da detecção facial seja consistente
em todo o código, evitando repetições e mantendo a coesão do sistema de detecção facial. */

// Chama a função app_mtmn_config() para obter uma estrutura mtmn_config_t configurada
mtmn_config_t mtmn_config = app_mtmn_config();

/* face_id_name_list st_face_list;: Esta linha declara uma variável chamada st_face_list do tipo face_id_name_list. Essa variável provavelmente é usada para armazenar
uma lista de IDs de faces e seus nomes associados.
static dl_matrix3du_t *aligned_face = NULL;: Aqui, declara-se um ponteiro chamado aligned_face do tipo dl_matrix3du_t, que é inicializado como NULL.
Esse ponteiro provavelmente é usado para apontar para uma matriz de dados de imagem alinhada durante o processo de reconhecimento facial.
O uso de static na declaração de aligned_face significa que essa variável terá uma duração de vida estática, ou seja, ela manterá seu valor entre
chamadas de função e será acessível apenas dentro do arquivo onde foi declarada. Isso pode ser útil para garantir que o ponteiro seja compartilhado entre diferentes
funções dentro do mesmo arquivo, por exemplo, no contexto de um sistema de reconhecimento facial.
 */

// Declaração de variáveis globais:
face_id_name_list st_face_list;             // Declara uma variável st_face_list do tipo face_id_name_list
static dl_matrix3du_t *aligned_face = NULL; // Declara um ponteiro aligned_face do tipo dl_matrix3du_t e o inicializa como NULL

/* httpd_handle_t: Este é um tipo de dados usado para representar um identificador (handle) para o servidor HTTP na plataforma ESP32.
camera_httpd: É o nome da variável que será utilizada para armazenar o identificador do servidor HTTP.
= NULL: A inicialização da variável como NULL significa que ela não está apontando para nenhum servidor HTTP no momento. Geralmente,
essa é uma prática comum para garantir que variáveis de identificador estejam inicializadas corretamente antes de serem utilizadas, evitando problemas de acesso a memória não alocada.
 */

// Declaração de uma variável do tipo httpd_handle_t para o servidor HTTP e a inicializa como NULL
httpd_handle_t camera_httpd = NULL;

/* typedef enum { ... } en_fsm_state;: Essa declaração cria um novo tipo de enumeração chamado en_fsm_state, que é usado para representar os diferentes estados em uma máquina de
estados finitos (FSM). Cada estado é associado a um valor numérico, mas também pode ser tratado de forma mais legível por meio dos identificadores definidos (START_STREAM, START_DETECT, etc.).
en_fsm_state g_state;: Aqui, é declarada uma variável global chamada g_state do tipo en_fsm_state, que é usada para armazenar o estado atual da máquina de estados.
typedef struct { ... } httpd_resp_value;: Essa declaração define uma estrutura chamada httpd_resp_value, que contém um único campo enroll_name, que é um array de caracteres
usado para armazenar o nome de uma inscrição (enrollment). Isso é útil para lidar com dados associados a respostas de requisições HTTP em um servidor web.
 */

// Definição de um novo tipo enum para representar estados de uma máquina de estados finitos (FSM)
typedef enum
{
  START_STREAM,      // Estado para iniciar o streaming de vídeo
  START_DETECT,      // Estado para iniciar a detecção de objetos ou faces
  SHOW_FACES,        // Estado para mostrar as faces detectadas
  START_RECOGNITION, // Estado para iniciar o reconhecimento facial
  START_ENROLL,      // Estado para iniciar o processo de inscrição (enrollment)
  ENROLL_COMPLETE,   // Estado para indicar a conclusão do processo de inscrição
  DELETE_ALL,        // Estado para excluir todos os dados (faces, etc.)
} en_fsm_state;

en_fsm_state g_state; // Variável global para armazenar o estado atual da máquina de estados

// Definição de uma estrutura para representar o valor de resposta de uma requisição HTTP
typedef struct
{
  char enroll_name[ENROLL_NAME_LEN]; // Array de caracteres para armazenar o nome de uma inscrição
} httpd_resp_value;

/* httpd_resp_value: Esse é o tipo de dados definido anteriormente por meio do typedef struct, que representa a estrutura de dados usada para armazenar informações de resposta HTTP,
como um nome de inscrição (enrollment_name).
st_name: É o nome da variável que será usada para armazenar informações de resposta HTTP, como o nome de uma inscrição durante o processamento de uma solicitação HTTP.
 */

// Declaração de uma variável do tipo httpd_resp_value para armazenar informações de resposta HTTP
httpd_resp_value st_name;

// Inicialização do Hardware e Conexão à Rede WiFi:
void setup()
{

  // Serial.begin(115200);: Esta linha inicializa a comunicação serial com uma taxa de transmissão de dados de 115200 bits por segundo (bps). Isso é comumente usado para estabelecer
  // uma conexão serial entre o dispositivo e o computador ou outro dispositivo para troca de dados.
  // Serial.setDebugOutput(true);: Essa linha habilita a saída de depuração serial, o que significa que as mensagens de depuração serão enviadas para a porta serial.
  // Isso é útil para depurar o código e monitorar variáveis, status ou mensagens importantes durante a execução do programa.
  // Serial.println();: Esta linha imprime uma nova linha na porta serial. Isso é útil para começar com uma linha vazia ou para separar visualmente as mensagens de
  // depuração que serão impressas posteriormente na porta serial.

  // Inicializa a conexão serial com uma taxa de transmissão de 115200 bps
  Serial.begin(115200);
  // Habilita a saída de depuração serial para enviar mensagens de depuração para a porta serial
  Serial.setDebugOutput(true);
  // Imprime uma nova linha na porta serial para começar com uma linha vazia
  Serial.println();

  // camera_config_t config;: Cria uma estrutura camera_config_t chamada config que será usada para configurar os parâmetros da câmera.
  // As linhas subsequentes configuram os diferentes pinos e canais para a interface da câmera, como pinos de dados (D0 a D7), pinos de clock (XCLK), pixel clock (PCLK), sincronização vertical (VSYNC), etc.
  // config.xclk_freq_hz = 20000000;: Define a frequência do clock da câmera em 20MHz (20000000 Hz), o que é uma frequência relativamente alta para garantir uma boa qualidade de imagem e desempenho.
  // config.pixel_format = PIXFORMAT_JPEG;: Define o formato de pixel da câmera como JPEG, indicando que as imagens capturadas pela câmera serão no formato JPEG, com compressão de imagem.
  // A observação "init with high specs to pre-allocate larger buffers" sugere que a configuração inicial está sendo feita com especificações altas para pré-alocar buffers maiores, o que pode
  // ser útil para lidar com dados de imagem de alta resolução ou processamento exigente.

  // Inicializa a câmera e configuração de hardware
  camera_config_t config;
  // Configura os pinos e canais para a interface da câmera
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;       // Frequência do clock da câmera em Hz (20MHz)
  config.pixel_format = PIXFORMAT_JPEG; // Formato de pixel JPEG

  // psramFound(): É uma função que verifica se a memória PSRAM está presente e disponível para uso no dispositivo ESP32.
  // config.frame_size e config.jpeg_quality: Essas configurações definem o tamanho do quadro de imagem e a qualidade JPEG das imagens capturadas pela câmera,
  // respectivamente. No caso da memória PSRAM estar disponível, a configuração de maior resolução e menor qualidade de compressão é usada, enquanto que no caso contrário,
  // uma configuração de menor resolução e qualidade de compressão ligeiramente melhor é usada.
  // config.fb_count: Define a quantidade de buffers de framebuffer que serão alocados para a câmera. Isso pode ser ajustado para otimizar o uso de memória,
  // mas geralmente mais buffers significam uma melhor capacidade de processamento e manipulação de imagens em tempo real.

  // Verifica se a memória PSRAM está disponível
  if (psramFound())
  {
    // Configuração para quando a memória PSRAM está disponível
    config.frame_size = FRAMESIZE_UXGA; // Tamanho do quadro de imagem definido como UXGA (1600x1200 pixels)
    config.jpeg_quality = 10;           // Qualidade JPEG definida como 10 (pode variar de 0 a 63, onde 0 é a menor qualidade e 63 é a melhor)
    config.fb_count = 2;                // Contagem de buffers de framebuffer definida como 2 (pode ser ajustada para otimização de memória)
  }
  else
  {
    // Configuração para quando a memória PSRAM não está disponível
    config.frame_size = FRAMESIZE_SVGA; // Tamanho do quadro de imagem definido como SVGA (800x600 pixels)
    config.jpeg_quality = 12;           // Qualidade JPEG definida como 12 (um pouco melhor em comparação com a configuração para memória PSRAM disponível)
    config.fb_count = 1;                // Contagem de buffers de framebuffer definida como 1
  }

//#if defined(CAMERA_MODEL_ESP_EYE): Esta diretiva de pré-processador verifica se o símbolo CAMERA_MODEL_ESP_EYE está definido. Isso geralmente é feito através de definições condicionais
// no arquivo de configuração ou em outras partes do código-fonte.
// pinMode(13, INPUT_PULLUP); e pinMode(14, INPUT_PULLUP);: Essas linhas configuram os pinos GPIO 13 e 14 do microcontrolador como entradas com resistores de pull-up ativados.
// Isso significa que, quando nenhum dispositivo externo estiver conectado a esses pinos, eles terão um estado lógico alto devido ao resistor de pull-up interno do microcontrolador.
// Essas diretivas de pré-processador são úteis para fazer o código se adaptar dinamicamente a diferentes configurações de hardware, garantindo que partes específicas do código sejam
// incluídas ou excluídas conforme necessário para diferentes modelos de dispositivos.

// Verifica se o modelo da câmera é ESP-EYE
#if defined(CAMERA_MODEL_ESP_EYE)
  // Configuração específica para o modelo ESP-EYE
  pinMode(13, INPUT_PULLUP); // Define o pino GPIO 13 como entrada com resistor de pull-up ativado
  pinMode(14, INPUT_PULLUP); // Define o pino GPIO 14 como entrada com resistor de pull-up ativado
#endif

  // esp_camera_init(&config): Esta linha inicializa a câmera com os parâmetros de configuração definidos anteriormente na estrutura config.
  // esp_err_t err: Declara uma variável para armazenar o valor de retorno da função esp_camera_init, que é do tipo esp_err_t, usado para indicar o status da operação (sucesso ou falha).
  // if (err != ESP_OK) { ... }: Este bloco condicional verifica se houve algum erro durante a inicialização da câmera. Se err não for igual a ESP_OK (que indica sucesso), significa que
  // ocorreu um erro. Nesse caso, uma mensagem de falha é impressa na porta serial com o código de erro correspondente e a função retorna imediatamente, encerrando a execução do código nesta parte.
  // Essa verificação de erro é importante para lidar com possíveis problemas durante a inicialização da câmera e garantir que o programa não prossiga com operações inválidas ou não confiáveis
  // caso a inicialização da câmera não seja bem-sucedida.

  // Inicializa a câmera com a configuração previamente definida
  esp_err_t err = esp_camera_init(&config);
  // Verifica se houve algum erro durante a inicialização da câmera
  if (err != ESP_OK)
  {
    // Se houver um erro, imprime uma mensagem de falha com o código de erro correspondente
    Serial.printf("Camera init failed with error 0x%x", err);
    // Retorna imediatamente, interrompendo a execução do programa nesta função
    return;
  }

  // sensor_t *s = esp_camera_sensor_get();: Esta linha obtém um ponteiro para a estrutura sensor_t, que é usada para representar e configurar o sensor da câmera no ESP32.
  // s->set_framesize(s, FRAMESIZE_QVGA);: Esta linha utiliza o ponteiro s para chamar o método set_framesize da estrutura sensor_t, configurando assim o tamanho do quadro
  // de imagem para o sensor da câmera. No exemplo dado, o tamanho do quadro é configurado como QVGA (320x240 pixels), mas isso pode variar dependendo das opções disponíveis
  // para o sensor da câmera e das configurações específicas do projeto.
  // Essa configuração define o tamanho do quadro de imagem que será capturado pela câmera, influenciando diretamente na resolução e qualidade das imagens obtidas.

  // Obtém um ponteiro para a estrutura sensor_t que representa o sensor da câmera
  sensor_t *s = esp_camera_sensor_get();
  // Configura o tamanho do quadro de imagem para o sensor da câmera para QVGA (320x240 pixels)
  s->set_framesize(s, FRAMESIZE_QVGA);

//#if defined(CAMERA_MODEL_M5STACK_WIDE): Esta diretiva de pré-processador verifica se o símbolo CAMERA_MODEL_M5STACK_WIDE está definido, indicando que o modelo da câmera é o "M5Stack Wide".
// sensor_t *s = esp_camera_sensor_get();: Obtém um ponteiro para a estrutura sensor_t, permitindo acessar e configurar as propriedades do sensor da câmera.
// s->set_vflip(s, 1);: Aplica um flip vertical ao sensor da câmera, ou seja, inverte a imagem na direção vertical. Isso pode ser útil para corrigir a orientação da imagem dependendo da posição da câmera.
// s->set_hmirror(s, 1);: Aplica um espelhamento horizontal ao sensor da câmera, ou seja, inverte a imagem na direção horizontal. Isso também pode ser útil para corrigir a orientação
// da imagem em certas situações.
// Essas configurações de flip e espelhamento são comuns em aplicações de câmera para ajustar a orientação e a aparência das imagens conforme necessário.

// Verifica se o modelo da câmera é M5Stack Wide
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  // Obtém um ponteiro para a estrutura sensor_t que representa o sensor da câmera
  sensor_t *s = esp_camera_sensor_get();
  // Aplica flip vertical ao sensor da câmera (inverte a imagem verticalmente)
  s->set_vflip(s, 1);
  // Aplica espelhamento horizontal ao sensor da câmera (inverte a imagem horizontalmente)
  s->set_hmirror(s, 1);
#endif

  // Inicializa o servidor HTTP e WebSocket

  if (!WiFi.config(local_IP, gateway, subnet))
  {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL};

// Essa função é responsável por inicializar o servidor HTTP e registrar um manipulador de URI para a página principal. Aqui está uma explicação mais detalhada:
// Configuração do Servidor HTTP:
// A função HTTPD_DEFAULT_CONFIG() retorna uma configuração padrão para o servidor HTTP, que inclui várias opções como porta, máximos de conexões, tamanho do buffer, entre outros.
// Essa configuração é atribuída à variável config.
// Inicialização do Servidor HTTP:
// A função httpd_start(&camera_httpd, &config) tenta iniciar o servidor HTTP com as configurações fornecidas.
// Se a inicialização for bem-sucedida (retorna ESP_OK), a mensagem "httpd_start" é impressa no monitor serial.
// Caso contrário, uma mensagem de erro pode ser impressa para indicar que a inicialização falhou.
// Registro de URI Handler:
// A função httpd_register_uri_handler(camera_httpd, &index_uri) registra um manipulador de URI para a página principal.
// Isso significa que quando uma requisição HTTP é feita para a URI "/", o manipulador index_handler será chamado para lidar com a requisição.

void app_httpserver_init()
{
  // Configuração padrão do servidor HTTP
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  // Inicia o servidor HTTP com as configurações padrão
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {
    // Se a inicialização for bem-sucedida, imprime uma mensagem
    Serial.println("httpd_start");
  }
  else
  {
    // Caso contrário, imprime uma mensagem de erro
    Serial.println("Failed to start httpd server");
  }

  // Registra um manipulador de URI para a página principal
  httpd_register_uri_handler(camera_httpd, &index_uri);
}

// Esta função é responsável por inicializar e preparar o sistema para o reconhecimento facial.
// Ela inicializa a lista de identificação facial com um número máximo de IDs e confirmações de enroll definidos pelas constantes FACE_ID_SAVE_NUMBER e ENROLL_CONFIRM_TIMES.
// Também aloca memória para uma matriz alinhada para a face e lê IDs de face previamente salvos na memória flash.

void app_facenet_main()
{
  // Inicializa a lista de identificação facial com o número máximo de IDs e confirmações de enroll
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);

  // Aloca memória para uma matriz alinhada para a face
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);

  // Lê IDs de face previamente salvos na memória flash
  read_face_id_from_flash_with_name(&st_face_list);
}

// Esta função inicia o processo de enroll de uma nova face na lista de identificação facial.
// Ela chama a função enroll_face_id_to_flash_with_name para realizar o enroll e retorna o número de amostras restantes para completar o enroll.

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  // Inicia o processo de enroll com o novo ID de face
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face; // Retorna o número de amostras restantes para o enroll
}

// Esta função envia a lista de faces para o cliente WebSocket.
// Primeiro, envia uma mensagem para o cliente informando para deletar todas as faces existentes.
// Em seguida, itera sobre a lista de faces (st_face_list) e envia cada face para o cliente no formato "listface:ID_da_face".
// client.send("delete_faces");: Esta linha envia a mensagem "delete_faces" para o cliente WebSocket. Parece ser uma instrução para o navegador ou cliente web 
// deletar todas as faces que tem.
// face_id_node *head = st_face_list.head;: Aqui, st_face_list.head parece ser um ponteiro para o primeiro nó de uma lista encadeada de faces. Este código 
// inicializa head com o ponteiro para o início da lista de faces.
// char add_face[64];: Cria um array de caracteres chamado add_face com tamanho 64. Este array é usado como um buffer para armazenar mensagens que serão enviadas para o navegador.
// O loop for itera sobre a lista de faces até st_face_list.count, que provavelmente é o número total de faces na lista.
// sprintf(add_face, "listface:%s", head->id_name);: Aqui, sprintf é usado para formatar a mensagem que será enviada para o navegador. Ele coloca o texto "listface:" seguido 
// do nome da face (head->id_name) dentro do array add_face.
// client.send(add_face);: Envia a mensagem formatada para o navegador através do cliente WebSocket.
// head = head->next;: Move head para o próximo nó da lista encadeada, preparando-o para a próxima iteração do loop e assim enviar a próxima face para o navegador.



void send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces"); // instrui o navegador a deletar todas as faces
  face_id_node *head = st_face_list.head; // obtém o ponteiro para o início da lista de faces
  char add_face[64]; // cria um buffer para armazenar a mensagem a ser enviada para o navegador

  // Itera sobre a lista de faces e envia cada face para o navegador
  for (int i = 0; i < st_face_list.count; i++)
  {
    // Formata a mensagem no formato "listface:<nome_da_face>" para enviar ao navegador
    sprintf(add_face, "listface:%s", head->id_name);

    // Envia a mensagem formatada para o navegador através do cliente WebSocket
    client.send(add_face);

    // Move para o próximo nó da lista de faces
    head = head->next;
  }
}


// Essa função chamada delete_all_faces parece ter a finalidade de excluir todas as faces armazenadas, utilizando uma função específica para deletar todas as faces na memória 
// flash e enviando uma mensagem para o cliente WebSocket informando sobre a exclusão das faces.
// delete_face_all_in_flash_with_name(&st_face_list);: Esta linha chama uma função (não fornecida no trecho de código) chamada delete_face_all_in_flash_with_name passando 
// o endereço de memória da lista de faces st_face_list como argumento. Presumivelmente, esta função é responsável por excluir todas as faces armazenadas na memória flash, 
// com base no nome da face ou algum critério específico.
// client.send("delete_faces");: Em seguida, esta linha envia a mensagem "delete_faces" para o cliente WebSocket, informando-o de que todas as faces foram deletadas.

void delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list); // exclui todas as faces na memória flash
  client.send("delete_faces"); // informa ao cliente WebSocket que as faces foram deletadas
}


// Essa função é essencial para interpretar e responder às mensagens recebidas do cliente WebSocket, ativando os estados apropriados no sistema e enviando mensagens de confirmação de volta ao cliente.

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
  // Verifica se a mensagem recebida é para iniciar o streaming
  if (msg.data() == "stream")
  {
    g_state = START_STREAM;   // Define o estado como iniciar streaming
    client.send("STREAMING"); // Envia mensagem de confirmação ao cliente
  }
  // Verifica se a mensagem recebida é para iniciar a detecção
  if (msg.data() == "detect")
  {
    g_state = START_DETECT;   // Define o estado como iniciar detecção
    client.send("DETECTING"); // Envia mensagem de confirmação ao cliente
  }
  // Verifica se a mensagem recebida é para capturar uma imagem
  if (msg.data().substring(0, 8) == "capture:")
  {
    g_state = START_ENROLL; // Define o estado como iniciar captura/enroll
    // Extrai o nome da pessoa a ser capturada da mensagem
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {
        0,
    };
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1); // Copia o nome para a estrutura de dados
    client.send("CAPTURING");                                // Envia mensagem de confirmação ao cliente
  }
  // Verifica se a mensagem recebida é para iniciar o reconhecimento
  if (msg.data() == "recognise")
  {
    g_state = START_RECOGNITION; // Define o estado como iniciar reconhecimento
    client.send("RECOGNISING");  // Envia mensagem de confirmação ao cliente
  }
  // Verifica se a mensagem recebida é para remover uma face
  if (msg.data().substring(0, 7) == "remove:")
  {
    // Extrai o nome da pessoa a ser removida da mensagem
    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));
    // Remove a face da lista de faces e envia a lista atualizada para o cliente
    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client); // Reseta as faces no navegador do cliente
  }
  // Verifica se a mensagem recebida é para deletar todas as faces
  if (msg.data() == "delete_all")
  {
    delete_all_faces(client); // Deleta todas as faces e envia mensagem de confirmação ao cliente
  }
}

void loop()
{
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
  http_img_process_result out_res = {0};
  out_res.image = image_matrix->item;

  send_face_list(client);
  client.send("STREAMING");

  while (client.available())
  {
    client.poll();

    fb = esp_camera_fb_get();

    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
    {
      out_res.net_boxes = NULL;
      out_res.face_id = NULL;

      fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);

      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);

      if (out_res.net_boxes)
      {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
        {

          out_res.face_id = get_face_id(aligned_face);
          last_detected_millis = millis();
          if (g_state == START_DETECT)
          {
            client.send("FACE DETECTED");
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "Enrolled Face ID: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "FACE CAPTURED FOR %s", st_face_list.tail->id_name);
              client.send(captured_message);
              send_face_list(client);
            }
          }

          if (g_state == START_RECOGNITION && (st_face_list.count > 0))
          {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f)
            {
              char recognised_message[64];
              sprintf(recognised_message, "RECOGNISED %s", f->id_name);
              client.send(recognised_message);
            }
            else
            {
              client.send("FACE NOT RECOGNISED");
            }
          }
          dl_matrix3d_free(out_res.face_id);
        }
      }
      else
      {
        if (g_state != START_DETECT)
        {
          client.send("NO FACE DETECTED");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500)
      { // Detecting but no face detected
        client.send("DETECTING");
      }
    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
