
// fd_forward.h: No such file or directory
// https://www.youtube.com/watch?v=knxe3zkd6rA
// esp32 em 1.0.6
// Firebase_ESP_Client em 2.3.7

#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include <esp_now.h>
#include <WiFi.h>
#include "SPIFFS.h"

// Estrutura para receber e enviar mensagens
typedef struct {
  char message_esp[32]; // Mensagem a ser recebida e enviada
} test_struct;

// Declaração da variável global
char espnow_message[32] = {0};  // Inicializada com zero

test_struct receivedData;  // Dados recebidosf
test_struct sendData = {"Hello from ESP32-2"};  // Mensagem inicial a ser enviada

// MAC Address do seu receptor
uint8_t partnerMacAddress[] = {0x30, 0xC9, 0x22, 0x27, 0xDA, 0xA8};

esp_now_peer_info_t peerInfo;


// Definição do nome do dispositivo
#define DEVICE_NAME "ESP32CAM"
#define LED_PIN 4  // Define o pino do LED

 IPAddress local_IP(10, 0, 0, 253);
 IPAddress gateway(10, 0, 0, 1);

IPAddress subnet(255, 255, 0, 0);

 const char *ssid = "INTELBRAS";
 const char *password = "Anaenena";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;

WebsocketsServer socket_server;

camera_fb_t *fb = NULL;

long current_millis;
long last_detected_millis = 0;

void app_facenet_main();
void app_httpserver_init();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}

mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

httpd_handle_t camera_httpd = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;

en_fsm_state g_state;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;


  WebsocketsClient connectedClient;  // Global client to send data to

#define MAX_RETRIES 3
int retry_count = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus do último pacote enviado:\t");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sucesso na entrega");
    retry_count = 0; // Reset retry count on successful send
  } else {
    Serial.println("Falha na entrega");
    if (retry_count < MAX_RETRIES) {
      retry_count++;
      Serial.print("Tentativa de reenvio #: ");
      Serial.println(retry_count);

      esp_now_send(partnerMacAddress, (uint8_t*)espnow_message, strlen(espnow_message));
    } else {
      Serial.println("Falha após várias tentativas.");
      retry_count = 0; // Reset retry count after max retries
    }
  }
}

// Função de callback para processar mensagens recebidas
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {

  Serial.print("Mensagem ESPNOW recebida de ");
  Serial.print(DEVICE_NAME);
  Serial.print(": ");
  Serial.println((char*)data);

  // Convertendo os dados recebidos para uma string para fácil comparação
  String dataStr = String((char*)data);

   // Encontra a posição do delimitador ':'
  int delimiterIndex = dataStr.indexOf(':');

  // Se o delimitador não for encontrado, imprime um erro e retorna
  if (delimiterIndex == -1) {
//    Serial.println("Delimitador ':' não encontrado na mensagem!");
    return;
  }

  // Extrai a parte do comando, que está antes do delimitador
  String command = dataStr.substring(0, delimiterIndex);

  // Extrai os dados adicionais, que estão depois do delimitador
  String additionalData = dataStr.substring(delimiterIndex + 1);
  additionalData.trim(); // Aplica trim para remover espaços iniciais

  Serial.print("Comando: ");
  Serial.println(command);
  Serial.print("Número do cartão: ");
  Serial.println(additionalData);
  

  // Verifica a mensagem e define o estado global g_state apropriadamente
  if (command == "start_stream") {
    g_state = START_STREAM;
  } else if (command == "start_detect") {
    g_state = START_DETECT;
  } else if (command == "show_faces") {
    g_state = SHOW_FACES;
  } else if (command == "start_recognition") {
    g_state = START_RECOGNITION;
  } else if (command == "capture") {

     // Send the message via WebSocket if the client is available

    handle_message1(connectedClient, String(dataStr));
    Serial.println("Websocket enviado");
    
//  }
  

  } else if (command == "enroll_complete") {
    g_state = ENROLL_COMPLETE;
  } else if (command == "delete_all") {
    g_state = DELETE_ALL;
  } else {
    Serial.println("Formato de mensagem inválido!");
    return; // Usar return para terminar a função se o formato for inválido
  }

  // Informa o estado atual baseado na mensagem recebida
  Serial.print("Estado atual: ");
  switch (g_state) {
    case START_STREAM:
      Serial.println("Iniciando transmissão...");
      break;
    case START_DETECT:
      Serial.println("Iniciando detecção...");
      break;
    case SHOW_FACES:
      Serial.println("Mostrando rostos...");
      break;
    case START_RECOGNITION:
      Serial.println("Iniciando reconhecimento...");
      break;
    case START_ENROLL:
      Serial.println("Iniciando cadastro...");
      break;
    case ENROLL_COMPLETE:
      Serial.println("Cadastro completo.");
      break;
    case DELETE_ALL:
      Serial.println("Deletando todos os dados.");
      break;
  }
}

esp_now_peer_info_t peerInfo;

void setup()
{

  Serial.begin(115200);

 if (!SD_MMC.begin()) {
    Serial.println("Falha ao inicializar o cartão SD");
    return;
  }
  Serial.println("Cartão SD inicializado");

  // Abre o arquivo para escrita
  File file = SD_MMC.open("/teste.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Erro ao abrir o arquivo");
    return;
  }

void setup()
{


  pinMode(LED_PIN, OUTPUT);  // Configura o pino do LED como saída

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully.");  // Log de sucesso

  logFileDetails(); // Listar arquivos e seus tamanhos
  logFileSystemInfo(); // Logar espaço total, usado e livre


  Serial.setDebugOutput(true);

  Serial.println();

  camera_config_t config;

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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {

    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {

    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)

  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {

    Serial.printf("Camera init failed with error 0x%x", err);

    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)

  sensor_t *s = esp_camera_sensor_get();

  s->set_vflip(s, 1);

  s->set_hmirror(s, 1);
#endif

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

  // Inicializar o ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar o ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, partnerMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Configurar a função de callback para receber mensagens
  esp_now_register_recv_cb(OnDataRecv);

  // Se chegou aqui, significa que o parceiro foi adicionado com sucesso.
  Serial.println("Parceiro adicionado com sucesso!");

  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);

  Serial.print("Câmera pronta! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' conectar");
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
  .user_ctx = NULL
};

void app_httpserver_init()
{

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  if (httpd_start(&camera_httpd, &config) == ESP_OK)
  {

    Serial.println("httpd_start");
  }
  else
  {

    Serial.println("Failed to start httpd server");
  }

  httpd_register_uri_handler(camera_httpd, &index_uri);
}

void app_facenet_main()
{

  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);

  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);

  Serial.println("Loading face data...");
  read_face_id_from_flash_with_name(&st_face_list);
  Serial.println("Face data loaded successfully.");

}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");

  Serial.println("Attempting to enroll new face...");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  Serial.println("Saving face data...");
  bool result = enroll_face_id_to_flash_with_name(&st_face_list, new_id, st_name.enroll_name);
  if (result) {
    Serial.println("Face data saved successfully.");

    logFileDetails();  // Chama a função para logar os detalhes dos arquivos
    logFileSystemInfo();  // Chama a função para logar as informações do sistema de arquivos


  } else {
    Serial.println("Error saving face data!");
  }



  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

void send_face_list(WebsocketsClient &client)
{
  client.send("delete_faces");
  face_id_node *head = st_face_list.head;
  char add_face[64];

  for (int i = 0; i < st_face_list.count; i++)
  {

    sprintf(add_face, "listface:%s", head->id_name);

    client.send(add_face);

    head = head->next;
  }
}

void delete_all_faces(WebsocketsClient &client)
{
  delete_face_all_in_flash_with_name(&st_face_list);
  client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{

  if (msg.data() == "stream")
  {
    g_state = START_STREAM;
    client.send("STREAMING");
  }

  if (msg.data() == "detect")
  {
    g_state = START_DETECT;
    client.send("DETECTING");
  }

  if (msg.data().substring(0, 8) == "capture:")
  {
    g_state = START_ENROLL;

    Serial.println("ESPNOW passa por aqui");

    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {
      0,
    };
    msg.data().substring(8).toCharArray(person, sizeof(person));
    memcpy(st_name.enroll_name, person, strlen(person) + 1);
    client.send("CAPTURING");
  }

  if (msg.data() == "recognise")
  {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  }

  if (msg.data().substring(0, 7) == "remove:")
  {

    char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
    msg.data().substring(7).toCharArray(person, sizeof(person));

    delete_face_id_in_flash_with_name(&st_face_list, person);
    send_face_list(client);
  }

  if (msg.data() == "delete_all")
  {
    delete_all_faces(client);
  }
}

void handle_message1(WebsocketsClient &client, const String &data) {
  if (data == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  } else if (data == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  } else if (data.startsWith("capture:")) {
    g_state = START_ENROLL;
     Serial.println("PASSED START_ENROLL");
//-----------
 // Alocando o array de caracteres para o nome da pessoa
    char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0};

    // Extraindo o nome da pessoa a partir do comando
    String personName = data.substring(8);
    personName.toCharArray(person, sizeof(person));

    // Copiando o nome para a variável de estrutura (assumindo que st_name é uma estrutura disponível)
    memcpy(st_name.enroll_name, person, strlen(person) + 1);

    // Enviando a resposta de captura para o cliente WebSocket
    client.send("CAPTURING");
    
//    -------------------
//    client.send("CAPTURING");
  } else if (data == "recognise") {
    g_state = START_RECOGNITION;
    client.send("RECOGNISING");
  } else if (data.startsWith("remove:")) {
    // Processamento específico de 'remove'
    client.send("REMOVED");
  } else if (data == "delete_all") {
    // Processamento específico de 'delete_all'
    client.send("ALL DELETED");
  } else {
    client.send("COMMAND UNKNOWN");
  }
}

void blinkLed() {
    digitalWrite(LED_PIN, HIGH);  // Acende o LED
    delay(200);                  // Mantém o LED aceso por 200 milissegundos
    digitalWrite(LED_PIN, LOW);   // Apaga o LED
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
            client.send("ROSTO DETECTADO");
          }

          if (g_state == START_ENROLL)
          {
            int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
            char enrolling_message[64];
            sprintf(enrolling_message, "NÚMERO DA AMOSTRA %d PARA %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
            client.send(enrolling_message);
            if (left_sample_face == 0)
            {
              ESP_LOGI(TAG, "ID facial inscrito: %s", st_face_list.tail->id_name);
              g_state = START_STREAM;
              char captured_message[64];
              sprintf(captured_message, "ROSTO CAPTURADO PARA %s", st_face_list.tail->id_name);
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
              sprintf(recognised_message, "RECONHECIDO %s", f->id_name);
              client.send(recognised_message);

              // Chame a função blinkLed() para piscar o LED
              blinkLed();


                  sprintf(espnow_message, "RECOGNISED: %s", f->id_name);  // Formata a string com o id_name

                  esp_err_t result = esp_now_send(partnerMacAddress, (uint8_t*)espnow_message, strlen(espnow_message));
  
                if (result == ESP_OK) {
                  Serial.println("Mensagem enviada para ESP32 com sucesso");
                  g_state = START_STREAM;
                  client.send("STREAMING");
                } else {
                  Serial.println("Erro ao enviar a mensagem para ESP32");
                }
              
            }
            else
            {
              client.send("ROSTO NÃO RECONHECIDO");
            }

  
          }
          dl_matrix3d_free(out_res.face_id);

        }
      }
      else
      {
        if (g_state != START_DETECT)
        {
          client.send("NENHUM ROSTO DETECTADO");
        }
      }

      if (g_state == START_DETECT && millis() - last_detected_millis > 500)
      {
        client.send("DETECTANDO");
      }
    }

    client.sendBinary((const char *)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;
  }

}
