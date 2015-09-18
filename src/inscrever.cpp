#include "ros/ros.h" // Incluir a biblioteca do ros
#include "geometry_msgs/Twist.h" //tipo de mensagem que sera enviada| wiki: http://wiki.ros.org/common_msgs?distro=indigo

/**
 * Exmplo de como criar um programa(node) para publicar uma mensagem em um topico do ROS
 */

/*
* Funcao chamada quando existe uma publicacao no topico ao qual o programa esta inscrito
* Recebe uma mensagem do tipo geometry_msgs/twist como parametro e imprime o que foi publicado
*/
void funcaoCallback(const geometry_msgs::Twist msg)
{
  //O ROS_INFO substitui o cout imprimindo a informacao na tela
  ROS_INFO("Estou Lendo linear X_:[%f] \n linear_Y:[%f] \n angular_[%f] \n", msg.linear.x, msg.linear.y, msg.angular.z);
}

//rotina principal do programa
int main(int argc, char **argv)
{

/**
  *  A funcao ros :: init () precisa ver argc e argv para que ele possa executar
    * quaisquer argumentos ROS e nome que foram fornecidos na linha de comando.
    * O terceiro argumento para init () é o nome do node.
    * Você deve chamar o ros :: init () antes de usar qualquer outra parte do sistema do ROS.
   */
  ros::init(argc, argv, "inscrito");

  /*  NodeHandle eh o principal ponto de acesso para a comunicacao
    * com o sistema do ROS. Ira inicializar totalmente este node e 
    * o último NodeHandle destruído irá limpar todos os recursos que o node estava usando*/
  ros::NodeHandle n;

   /*Funcao para inscrever em um topico. O ROS chama a funcaoCallback, sempre que chegar uma nova mensagem
    *O Segundo argumento eh o tamanho da fila, nesse caso, quando nao for possivel processar as mensagens rapido o suficiente
    * a fila alcanca 1000 mensagens, descartando mensagens antigas quando as novas chegarem*/
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, funcaoCallback);
  /*
  * ros::spin() introduz um ciclo, chamando mensagem callback tão rápido quanto possivel. 
  * Se não houver nada não vai usar muito a CPU. quando o roscore for desligado, ele ira encerrar
  */
  ros::spin(); //

  return 0;
}
