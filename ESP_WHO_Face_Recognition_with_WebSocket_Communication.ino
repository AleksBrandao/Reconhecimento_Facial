// Inclusão de bibliotecas necessárias
#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include  "fd_forward.h" 
#include  "fr_forward.h " 
#include  "fr_flash.h" 

// Constantes para conexão WiFi e configurações de reconhecimento facial
const char* ssid = "INTELBRAS";
const char* password = "Anaenena";
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// Funções e variáveis globais
WebsocketsServer socket_server;
camera_fb_t * fb = NULL;
long current_millis;
long last_detected_millis = 0;

// Configuração da câmera
void setup() {
    // Inicializa a conexão serial para debug
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // Configuração da câmera
    camera_config_t config;
    // ...
    esp_err_t err = esp_camera_init(&config);
    // ...

    // Inicialização da conexão WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    // Inicialização do servidor HTTP e WebSocket
    app_httpserver_init();
    app_facenet_main();
    socket_server.listen(82);

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
}

// Loop principal
void loop() {
    // Aceita conexão WebSocket
    auto client = socket_server.accept();
    client.onMessage(handle_message);
    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
    http_img_process_result out_res = {0};
    out_res.image = image_matrix->item;

    // Envia a lista de faces para o cliente WebSocket e inicia o streaming
    send_face_list(client);
    client.send("STREAMING");

    // Loop de processamento do cliente WebSocket
    while (client.available()) {
        client.poll();
        fb = esp_camera_fb_get();
        // ...
        // Processa o reconhecimento facial e envia dados para o cliente WebSocket
        // ...
        client.sendBinary((const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        fb = NULL;
    }
}

// Função de tratamento de mensagens WebSocket
void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
    // Verifica o tipo de mensagem recebida e realiza ações correspondentes
    if (msg.data() == "stream") {
        // Inicia o streaming
        client.send("STREAMING");
    }
    // ...
    // Outras condições e ações de acordo com as mensagens recebidas
    // ...
}

// Função de inicialização do servidor HTTP e definição de URIs
void app_httpserver_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
        Serial.println("httpd_start");
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
    }
}

// Função de inicialização do reconhecimento facial
void app_facenet_main() {
    // Inicializa estruturas e variáveis relacionadas ao reconhecimento facial
    face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    read_face_id_from_flash_with_name(&st_face_list);
}

// Função de processamento de mensagem WebSocket para enviar a lista de faces para o cliente
void send_face_list(WebsocketsClient &client) {
    client.send("delete_faces"); // tell browser to delete all faces
    face_id_node *head = st_face_list.head;
    char add_face[64];
    for (int i = 0; i < st_face_list.count; i++) // loop current faces
    {
        sprintf(add_face, "listface:%s", head->id_name);
        client.send(add_face); //send face to browser
        head = head->next;
    }
}

// Função para excluir todas as faces
void delete_all_faces(WebsocketsClient &client) {
    delete_face_all_in_flash_with_name(&st_face_list);
    client.send("delete_faces");
}
