#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"
#include "aws_iot_version.h"

#include "aws_custom_utils.h"
#include "output_driver.h"

#define TAG "CLOUD"
#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 200
#define MAX_DESIRED_PARAM 4
#define MAX_REPORTED_PARAM 4
#define NUM_OF_RELAYS 4

/*
 * The Json Document in the cloud will be:
 * {
 *   "reported": {
 *      "relay_1": true,
 *      "relay_2": true,
 *      "relay_3": true,
 *      "relay_4": true
 *    },
 *   "desired": {
 *      "relay_1": true,
 *      "relay_2": true,
 *      "relay_3": true,
 *      "relay_4": true
 *   }
 * }
 */

/* Per-Device Unique components:
 * - Device ID
 * - Certificate
 * - Private Key
 */

// thing name or device id
extern const uint8_t deviceid_txt_start[] asm("_binary_deviceid_txt_start");
extern const uint8_t deviceid_txt_end[] asm("_binary_deviceid_txt_end");

// Device cert and private key
extern const uint8_t
    certificate_pem_crt_start[] asm("_binary_device_cert_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_device_cert_end");
extern const uint8_t private_pem_key_start[] asm("_binary_device_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_device_key_end");
// Root Certificate
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_server_cert_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_server_cert_end");

// AWS IoT Endpoint specific to account and region
extern const uint8_t endpoint_txt_start[] asm("_binary_endpoint_txt_start");
extern const uint8_t endpoint_txt_end[] asm("_binary_endpoint_txt_end");

static bool reported_state[4] = {false, false, false, false};
unsigned short relay_number[4] = {1, 2, 3, 4};

// creating output state change callback
static bool output_changed_locally[4] = {false, false, false, false};
static void output_state_change_callback_1(const char *pJsonString,
                                           uint32_t JsonStringDataLen,
                                           jsonStruct_t *pContext) {
  if (pContext != NULL) {
    bool state = *(bool *)(pContext->pData);
    ESP_LOGI(TAG, "Delta - Output state changed to %s",
             state ? "true" : "false");
    app_driver_set_state(state, relay_number[0]);
    output_changed_locally[0] = false;
  }
}

static void output_state_change_callback_2(const char *pJsonString,
                                           uint32_t JsonStringDataLen,
                                           jsonStruct_t *pContext) {
  if (pContext != NULL) {
    bool state = *(bool *)(pContext->pData);
    ESP_LOGI(TAG, "Delta - Output state changed to %s",
             state ? "true" : "false");
    app_driver_set_state(state, relay_number[1]);
    output_changed_locally[1] = false;
  }
}

static void output_state_change_callback_3(const char *pJsonString,
                                           uint32_t JsonStringDataLen,
                                           jsonStruct_t *pContext) {
  if (pContext != NULL) {
    bool state = *(bool *)(pContext->pData);
    ESP_LOGI(TAG, "Delta - Output state changed to %s",
             state ? "true" : "false");
    app_driver_set_state(state, relay_number[2]);
    output_changed_locally[2] = false;
  }
}

static void output_state_change_callback_4(const char *pJsonString,
                                           uint32_t JsonStringDataLen,
                                           jsonStruct_t *pContext) {
  if (pContext != NULL) {
    bool state = *(bool *)(pContext->pData);
    ESP_LOGI(TAG, "Delta - Output state changed to %s",
             state ? "true" : "false");
    app_driver_set_state(state, relay_number[3]);
    output_changed_locally[3] = false;
  }
}

// creating output state change callback
static bool shadowUpdateInProgress;
static void update_status_callback(const char *pThingName,
                                   ShadowActions_t action,
                                   Shadow_Ack_Status_t status,
                                   const char *pReceivedJsonDocument,
                                   void *pContextData) {
  IOT_UNUSED(pThingName);
  IOT_UNUSED(action);
  IOT_UNUSED(pReceivedJsonDocument);
  IOT_UNUSED(pContextData);

  shadowUpdateInProgress = false;

  if (SHADOW_ACK_TIMEOUT == status) {
    ESP_LOGE(TAG, "Update timed out");
  } else if (SHADOW_ACK_REJECTED == status) {
    ESP_LOGE(TAG, "Update rejected");
  } else if (SHADOW_ACK_ACCEPTED == status) {
    ESP_LOGI(TAG, "Update accepted");
  }
}

// Creating shadow update
static IoT_Error_t shadow_update(AWS_IoT_Client *mqttClient,
                                 jsonStruct_t **reported_handles,
                                 size_t reported_count,
                                 jsonStruct_t **desired_handles,
                                 size_t desired_count) {
  IoT_Error_t rc = FAILURE;
  char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
  size_t sizeOfJsonDocumentBuffer =
      sizeof(JsonDocumentBuffer) / sizeof(JsonDocumentBuffer[0]);

  // Initialize JSON document with null terminated string
  rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer,
                                         sizeOfJsonDocumentBuffer);
  if (rc != SUCCESS) {
    return rc;
  }

  // fill json document with the reported

  if (reported_count > 0) {
    rc = custom_aws_iot_shadow_add_reported(JsonDocumentBuffer,
                                            sizeOfJsonDocumentBuffer,
                                            reported_count, reported_handles);
    if (rc != SUCCESS) {
      return rc;
    }
  }

  // fill json document with the desired
  if (desired_count > 0) {
    rc = custom_aws_iot_shadow_add_desired(JsonDocumentBuffer,
                                           sizeOfJsonDocumentBuffer,
                                           desired_count, desired_handles);
    if (rc != SUCCESS) {
      return rc;
    }
  }

  // Finalize JSON file
  rc = aws_iot_finalize_json_document(JsonDocumentBuffer,
                                      sizeOfJsonDocumentBuffer);
  if (rc != SUCCESS) {
    return rc;
  }

  // after finalizing the JSON file
  // update the shadow
  ESP_LOGI(TAG, "Updated Shadow: %s", JsonDocumentBuffer);
  rc = aws_iot_shadow_update(mqttClient, (const char *)deviceid_txt_start,
                             JsonDocumentBuffer, update_status_callback, NULL,
                             4, true);
  if (SUCCESS != rc) {
    return rc;
  }
  shadowUpdateInProgress = true;
  return rc;
}

// Create aws iot task

void aws_iot_task(void *param) {
  IoT_Error_t rc = FAILURE;
  bool output_state[4] = {false, false, false, false};
  AWS_IoT_Client mqttClient;

  // Create Shadow Parameter
  ShadowInitParameters_t sp = ShadowInitParametersDefault;
  sp.pHost = (char *)endpoint_txt_start;
  sp.port = AWS_IOT_MQTT_PORT;
  sp.pClientCRT = (const char *)certificate_pem_crt_start;
  sp.pClientKey = (const char *)private_pem_key_start;
  sp.pRootCA = (const char *)aws_root_ca_pem_start;
  sp.enableAutoReconnect = false;
  sp.disconnectHandler = NULL;

  // Initialize shadow
  ESP_LOGI(TAG, "Shadow Init");
  rc = aws_iot_shadow_init(&mqttClient, &sp);
  if (SUCCESS != rc) {
    ESP_LOGE(TAG, "Failed to initialize shadow %d", rc);
    goto error;
  }

  // Create shadow connect parameters
  ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
  scp.pMyThingName = (const char *)deviceid_txt_start;
  scp.pMqttClientId = (const char *)deviceid_txt_start;
  scp.mqttClientIdLen = (uint16_t)strlen((const char *)deviceid_txt_start);

  // Connecting to Thing
  ESP_LOGI(TAG, "Connecting to AWS Thing");
  do {
    rc = aws_iot_shadow_connect(&mqttClient, &scp);
    if (SUCCESS != rc) {
      ESP_LOGE(TAG, "Error (%d) connecting to %s: %d", rc, sp.pHost, sp.port);
      vTaskDelay(1000 / portTICK_RATE_MS);
    }
  } while (SUCCESS != rc);

  // enbable autoreconnect if a disconnection happens

  rc = aws_iot_shadow_set_autoreconnect_status(&mqttClient, true);
  if (SUCCESS != rc) {
    ESP_LOGE(TAG, "Unable to set Autorecoonect to true- %d", rc);
    goto aws_error;
  }

  // Creating a JSON structure for output
  jsonStruct_t output_handler[NUM_OF_RELAYS];
  const char *output[NUM_OF_RELAYS] = {"relay_1", "relay_2", "relay_3",
                                       "relay_4"};

  output_handler[0].cb = output_state_change_callback_1;
  output_handler[1].cb = output_state_change_callback_2;
  output_handler[2].cb = output_state_change_callback_3;
  output_handler[3].cb = output_state_change_callback_4;

  for (int i = 0; i < NUM_OF_RELAYS; i++) {
    output_state[i] = app_driver_get_state(relay_number[i]);
    output_handler[i].pData = &output_state[i];
    output_handler[i].dataLength = sizeof(output_state[i]);
    output_handler[i].type = SHADOW_JSON_BOOL;
    output_handler[i].pKey = output[i];

    rc = aws_iot_shadow_register_delta(&mqttClient, &output_handler[i]);
    if (SUCCESS != rc) {
      ESP_LOGE(TAG, "Shadow Register State Delta Error %d", rc);
      goto aws_error;
    }
  }

  // dynamic memory allocation for desired state and reported state
  // desired state
  jsonStruct_t **desired_handles =
      malloc(MAX_DESIRED_PARAM * sizeof(jsonStruct_t *));
  if (desired_handles == NULL) {
    ESP_LOGE(TAG, "Memory Allocation Failed");
    goto aws_error;
  }

  // reported state
  jsonStruct_t **reported_handles =
      malloc(MAX_REPORTED_PARAM * sizeof(jsonStruct_t *));
  if (reported_handles == NULL) {
    ESP_LOGE(TAG, "Memory Allocation Failed");
    free(desired_handles);
    goto aws_error;
  }

  // Fill the allocated json memory block with data
  // Report initial values once
  size_t desired_count = 0, reported_count = 0;

  for (int i = 0; i < NUM_OF_RELAYS; i++) {
    reported_handles[reported_count++] = &output_handler[i];
  }

  // update device shadow
  rc = shadow_update(&mqttClient, reported_handles, reported_count,
                     desired_handles, desired_count);
  for (int i = 0; i < NUM_OF_RELAYS; i++) {
    reported_state[i] = output_state[i];
  }

  while (NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc ||
         SUCCESS == rc) {
    rc = aws_iot_shadow_yield(&mqttClient, 200);
    if (NETWORK_ATTEMPTING_RECONNECT == rc || shadowUpdateInProgress) {
      rc = aws_iot_shadow_yield(&mqttClient, 1000);
      // If the client is attempting to reconnect, or already waiting on a
      // shadow update, we will skip the rest of the loop.
      continue;
    }

    desired_count = 0;
    reported_count = 0;

    // check output driver
    // changed in this is a local change of state
    for (int i = 0; i < NUM_OF_RELAYS; i++) {
      output_state[i] = app_driver_get_state(relay_number[i]);
      if (reported_state[i] != output_state[i]) {
        reported_handles[reported_count++] = &output_handler[i];
        if (output_changed_locally[i] == true) {
          desired_handles[desired_count++] = &output_handler[i];
        }
        output_changed_locally[i] = true;
        reported_state[i] = output_state[i];
      }
    }

    if (reported_count > 0 || desired_count > 0) {
      rc = shadow_update(&mqttClient, reported_handles, reported_count,
                         desired_handles, desired_count);
    }

    vTaskDelay(1000 / portTICK_RATE_MS);
  }

  if (SUCCESS != rc) {
    ESP_LOGE(TAG, "An error occured in the loop %d", rc);
  }

  if (reported_handles) {
    free(reported_handles);
  }
  if (desired_handles) {
    free(desired_handles);
  }

  // aws error

aws_error:
  ESP_LOGI(TAG, "Disconnecting");
  rc = aws_iot_shadow_disconnect(&mqttClient);

  if (SUCCESS != rc) {
    ESP_LOGE(TAG, "Disconnect error %d", rc);
  }
error:
  vTaskDelete(NULL);
}

int cloud_start(void) {
  printf("Starting cloud\n");

  BaseType_t cloud_begin =
      xTaskCreate(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL);
  if (cloud_begin != pdPASS) {
    ESP_LOGE(TAG, "Couldnt create a cloud task\n");
  }
  return ESP_OK;
}