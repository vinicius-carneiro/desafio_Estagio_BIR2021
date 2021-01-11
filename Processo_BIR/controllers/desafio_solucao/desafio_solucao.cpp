// Inclusão de arquivos necessários de cabeçalho da linguagem e do simulador
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <cstring>
#include <cmath>

// Diretivas de pré-processamento para valores de referência aplicados no Controller
// Antes da compilação do código, os valores dessas definições serão substituídos
#define TIME_STEP 8
#define MAX_SPEED 5.24
#define MAX_SENSOR_NUMBER 8
#define MAX_SENSOR_VALUE 1024
#define WHEEL_WEIGHT_THRESHOLD 100
#define MIN_DISTANCE 1.0

// namespace com palavras reservadas pelo programa Webots para utilizar em controllers em C++
using namespace webots;

// definição de uma struct para cálculo ponderado para os valores medidos pelos sonares
typedef struct {  
  double wheel_weight[2];
} SensorData;

// definição das palavras de mudança de chave, interpretadas na State Machine 
typedef enum { FORWARD, LEFT, RIGHT } State;

// definição dos valores ponderados para cada sensor, isto é, os sensores frontais e mais centralizados possuem maior peso
static SensorData sensors[MAX_SENSOR_NUMBER] = {
  {.wheel_weight = {150, 0}}, {.wheel_weight = {200, 0}}, {.wheel_weight = {300, 0}}, {.wheel_weight = {600, 0}},
  {.wheel_weight = {0, 600}}, {.wheel_weight = {0, 300}}, {.wheel_weight = {0, 200}}, {.wheel_weight = {0, 150}}};

//definição de variáveis constantes, entre elas as coordenadas X e Z do alvo a ser atingido
const double *centralGPS_position;
const double TargetX = -4.51866;
const double TargetZ = 2.36419;

int main(int argc, char **argv) {
  // Criação da instância do robô.
  // O simulador utiliza do paradigam de POO da linguagem C++, instânciando na memória Heap um objeto robot da classe Robot
  Robot *robot = new Robot();
  
  // Inicia a variável para leitura dos sensores. Apenas os 8 sonares frontais utilizados pelo controle
  DistanceSensor *ps[MAX_SENSOR_NUMBER];
  // Como o controlador precisa acessar os nós modelados no simulador, as funções do tipo get necessitam do nome do nó do dispositivo a ser acessado
  char psNames[MAX_SENSOR_NUMBER][5] = {
  "so0", "so1", "so2", "so3",
  "so4", "so5", "so6", "so7" 
  };  
  // Loop para habilitar o controle de cada sonar utilizando os nomes armazenadas na matriz psNames acima
  for (int i = 0; i < MAX_SENSOR_NUMBER; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
  }
  
  // Inicialização dos atuadores direito e esquerdo do Pioneer3DX.
  Motor *leftMotor = robot->getMotor("left wheel");
  Motor *rightMotor = robot->getMotor("right wheel");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // Inicialização do GPS   
  GPS *centralGPS = robot->getGPS("central GPS");
  centralGPS->enable(TIME_STEP);  
  
  // Declação de variáveis dinâmicas utilizadas no loop de controle
  double speed[2] = {0.0, 0.0};
  double wheel_weight_total[2] = {0.0, 0.0};
  double distance, speed_modifier, sensor_value;
  double goal_distance;
  
  // Por padrão o robô inicia em FORWARD, até quando necessário alterar o o controle switch para fazer o desvio de algum obstáculo 
  State state = FORWARD;
  
  // Loop principal de controle. O Controller é executado até quando a simulação for encerrada.
  while (robot->step(TIME_STEP) != -1) {
    
    // Pré-alocamento de memória para esses vetores 
    memset(speed, 0, sizeof(double) * 2);
    memset(wheel_weight_total, 0, sizeof(double) * 2);
    
    
    // Aqui, são lidos os valores de cada sonar. Caso o valor de um determinado sonar não satisfizer o mímimo necessário para ser considerado, obstáculo 
    // ainda tem uma certa distância, a speed_modifier é 0. 
    //Caso contrário, a distância é calculada através da lookup table definida, que correlaciona os valores de 0 a 1024 medidos pelos sensores com 
    // uma distância em metros ao obstáculo. Se a distância for inferior ao limite mínimo definido, speed_modifier guarda um valor proporcional a distância que falta
    // para colidir 
    for (int i = 0; i < MAX_SENSOR_NUMBER; ++i) {
      sensor_value = ps[i]->getValue();
      
     
      if (sensor_value == 0.0)
        speed_modifier = 0.0;
      else {
       
        distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE));  // lookup table inverse.
        // No código de controle original do desafio escrito em C, a fórmula resultava em valores
        // muito grandes, o que implicava em erro
        
        
        if (distance < MIN_DISTANCE)
          speed_modifier = 1 - (distance / MIN_DISTANCE);
        else
          speed_modifier = 0.0;
      }

      // incrementa a variável de peso total correspondente a cada roda (j=0 -> esquerda; j=1 -> direita),
      // a partir do peso que define a inflência de cada sensor (so0 a so7) para cada roda e vezes o speed_modifier
      // correspondente a cada sensor, dentro de uma step da simulação
      for (int j = 0; j < 2; ++j){
        wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier;
        }
      }    
      
      // Extrai a localização do robô no mundo a partir do sistema de coordenadas global adotado
      // A distância entre o robô e o alvo é calculada por trigonometria utilizando as coordenadas do alvo 
      // e as coordenadas do GPS no plano XZ
       centralGPS_position = centralGPS->getValues(); 
       goal_distance = sqrt(pow(TargetZ - centralGPS_position[2],2) + pow(TargetX - centralGPS_position[0],2));
      
      // Caso o robô tenha atingido uma distância mínima à luminária, o controlador é encerrado
       if (goal_distance < 0.6)
       {
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(0);
          printf("\n Alvo Atingido! \n");
          break;
       }


    // Aqui é utilizada a técnica de State Machine para alterar o comportamento do robô a partir das 3 chaves de estado pré-definidas com base
    // no cálculo ponderado das detecções de obstáculo de cada sensor e comparações com o limite estabelecido para mudança de estados
        
    switch (state) {
     // Caso os sensores do lado esquerdo do robô atinjam o limite mínimo de distânciamento para um obstáculo, de antemão, são modificados os 
     // valores de velocidade de modo que o sentido de rotação das rodas resultam em uma manobra à esquerda e o comportamento é alterado
     //para LEFT. Se ao invés ocorrer com o lado direito, o robô manobra para o lado direito e o estado passa para RIGHT. Nenhuma dessas duas condições sendo atingidas,
     // conclui-se que o caminho está livre e o robô segue em frente mantendo comportamento FORWARD.
     // A constante que multiplica MAX_SPEED é reduzida com relação a do código original do desafio
      case FORWARD:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
          state = RIGHT;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
        }
        break;
        
      // Com a swith configurada para LEFT (ou RIGHT), é verificado se um dos dois lados detectam objeto para manter o movimento 
      // de manobra à esquerda (ou à direita), até que esteja livre e o comportamento volte a ser FORWARD. A condição é testada com lógica
      // OU para evitar que em que em algum trecho do trajeto, por exemplo entre dois obstáculos próximos dos dois lados, o robô caia em um loop, 
      // alternando indeterminadamete entre as chaves LEFT e RIGHT.
      case LEFT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = 0.7 * MAX_SPEED;
          speed[1] = -0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break;
      case RIGHT:
        if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD) {
          speed[0] = -0.7 * MAX_SPEED;
          speed[1] = 0.7 * MAX_SPEED;
        } else {
          speed[0] = MAX_SPEED;
          speed[1] = MAX_SPEED;
          state = FORWARD;
        }
        break; 
    }
    
      // A saída da State Machine são os valores de velocidade para cada um dos atuadores do PIONEER
      leftMotor->setVelocity(speed[0]);
      rightMotor->setVelocity(speed[1]);
  }
  delete robot;
  // É deletado o objeto que foi criado, uma vez que o instanciamento foi na memória Heap, 
  // não se dá de forma automática ao encerrar o programa, devendo-se usar delete.
  return 0;
  // O Controller encerra com sucesso
}
