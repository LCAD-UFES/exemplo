#include "ros/ros.h" // Incluir a biblioteca do ros
#include "geometry_msgs/Twist.h" //tipo de mensagem que sera enviada| wiki: http://wiki.ros.org/common_msgs?distro=indigo

/**
 * Exmplo de como criar um programa(node) para publicar uma mensagem em um topico do ROS
 */
int main(int argc, char **argv)
{
    /**
    *  A funcao ros :: init () precisa ver argc e argv para que ele possa executar
    * quaisquer argumentos ROS e nome que foram fornecidos na linha de comando.
    * O terceiro argumento para init () é o nome do node.
    * Você deve chamar o ros :: init () antes de usar qualquer outra parte do sistema do ROS.
    */
    ros::init(argc, argv, "publicador"); //Inicializando ROS com nome do NODE

    /*  NodeHandle é o principal ponto de acesso para a comunicação
    * com o sistema do ROS. Irá inicializar totalmente este node e 
    * o último NodeHandle destruído irá limpar todos os recursos que o node estava usando*/
    ros::NodeHandle n; 

    /**
        * A funcao advertise() eh como você diz ao ROS o que voce quer publicar em um dado topico.
        * Isso invoca uma chamada ao node mestre ROS (roscore), que mantem um registro de quem  
        * esta publicando e quem esta se inscrevendo
        * Após a chamada do advertise(), o mestre (roscore) irá notificar qualquer 
        * um que está tentando assinar este nome do tópico,
        * E eles, por sua vez negociaam uma conexão peer-to-peer com este Node.
        * advertise() retorna um objeto do Publisher, que lhe permite
        * Publicar mensagens sobre o assunto atraves de uma chamada do publish(). 
        * Quando Todas as cópias do objeto Publisher devolvidos são destruídas, o topico
        * será automaticamente propagado.
    *
    * O segundo parametro para o advertise() é o tamanho do buffer de mensagem usado para publicar
    * Se mensagens sao publicadas mais rapidamente que enviadas, este numero especifica quantas mensagens 
    * sao colocadas no buffer antes de ignorar as outras
    */
    ros::Publisher velocidade_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(10); //permite que você especifique uma frequência de quanto tempo ele ira executar. Neste caso, 10Hz.


    geometry_msgs::Twist mensagem; //objeto para a mensagem

    while (ros::ok())
    {
        /**
            * A geometry_msg/Twist expressa a velocidade no espaço dividido em linear e angular
            * Aqui atribuimos a velocidade linear x e y e a angular Z
            */
        mensagem.linear.x = 0.2; 
        mensagem.linear.y = 0;
        mensagem.angular.z = 0.3;
        velocidade_pub.publish(mensagem); //Publica a mensagem
        /*O ROS_INFO substitui o cout imprimindo a informacao na tela*/
        ROS_INFO("Estou Publicando \n linear X_:[%f] \n linear_Y:[%f] \n angular_[%f] \n", mensagem.linear.x, mensagem.linear.y, mensagem.angular.z); 

        loop_rate.sleep(); //Usado para aguardar o tempo escolhido
    }

    return 0;
}
