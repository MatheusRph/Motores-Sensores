    while(counter == 0){
        Ne();
        //Se esquerda for branco, centro preto, direita branco
        if (LDR1 == 0 && LDR2 == 1 && LDR3 == 0){
            stopCar(300);
            while (true)
            {
                moveForward();
                fixrote();
                if( ultra1 <= X && ultra2 <= X){ //Logica Arvore grande
                    stopCar(300);
                    //Movimentação dos servos (Pegar a arvore e depois erguer a garra)
                    //pegarArvore();
                    while (true)
                    {
                        moveForward();
                        fixrote();
                        if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                            stopMotors();
                            //desce a arvore
                            //ergue a garra
                            while (true)
                            {
                                moveBackward();
                                fixrote();
                                if (ultra1 <= X && ultra2 >= X){
                                    stopMotors();
                                    //pegar arvore pegarArvore();
                                    while (true)
                                    {
                                        moveForward();
                                        fixrote();
                                        if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                                            stopMotors();
                                            //desce a arvore
                                            //ergue a garra
                                        }
                                    }
                                }
                            }
                        }
                    }
                    
                    // if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                    //     stopMotors();
                    // }
                    arvore grande

                } else if ( ultra1 <= X && ultra2 >= X){
                    stopCar(300);
                    //Movimentação dos servos (Pegar a arvore e depois erguer a garra)
                    //pegarArvore();
                    if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                        stopCar(300);
                    }
                    arvore pequena
                } else{
                    Serial.print("Erro ao encontrar arvores")
                    break; // Sai do loop se não encontrar árvores
                }
            }
        }
    }