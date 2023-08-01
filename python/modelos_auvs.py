import pandas as pd

class ModeloAUV():
    '''class que representa um modelo de AUV com seus atributos.'''
    def __init__(self,numThrusters:int,sensores:list,ano_construcao:int,nome:str,modelo_camera:str):
        #Definindo os atributos de um auv baseado nos argumentos passados.
        self.__numThrusters = numThrusters
        self.__sensores = sensores
        self.__ano_construcao = ano_construcao
        self.__nome = nome
        self.__modelo_camera = modelo_camera
    #Criando getters para acessar os atributos.
    def get_numThrusters(self):
        '''getter que retorna o numero de thrusters'''
        return self.__numThrusters
    def get_sensores(self):
        '''getter que retorna a lista de sensores'''
        return self.__sensores
    def get_ano_construcao(self):
        '''getter que retorna o ano de construcao'''
        return self.__ano_construcao
    def get_nome(self):
        '''getter que retorna o nome'''
        return self.__nome
    def get_modelo_camera(self):
        '''getter que retorna o modelo da camera'''
        return self.__modelo_camera


class exibicao():
    '''class que possui métodos para exibir informações na tela.'''
    def mostrar_tabela(self):
        '''metodo que mostra uma tabela com as informacoes dos dois auvs.'''
        #usa o objeto DataFrame do pandas para criar uma tabela com as informacoes.
        tabela = pd.DataFrame({'Numero de Thrusters':[brhue.get_numThrusters(),lua.get_numThrusters()],
                     'Ano de Construcao':[brhue.get_ano_construcao(),lua.get_ano_construcao()],
                     'Modelo das Cameras':[brhue.get_modelo_camera(),lua.get_modelo_camera()],
                     'Sensores':[brhue.get_sensores(),lua.get_sensores()]},
                    index=['BrHUE','Lua'])
        #printa a tabela na tela.
        print(tabela)
    
    def mostrar_robos(self):
        '''metodo que mostra as informacoes de um dos atributos.'''
        print('Os AUVs disponiveis sao 1-BrHUE e 2-Lua.')
        #pergunta de qual modelo o usuario quer ler as informacoes.
        opcao = input('Voce quer ver a informacao de qual deles?(1/2) ')
        while opcao!='1' and opcao!='2':
            #insiste na pergunta ate o usuario digitar '1' ou '2'.
            opcao = input('Voce quer ver a informacao de qual deles?(1/2) ')
        #baseado no que o usuario digitar, pega um dos modelos de auv.
        robos = {'1':brhue,'2':lua}
        auv = robos[opcao]
        #pega o nome do auv e imprime na tela.
        nome = auv.get_nome()
        print(f'Nome do auv:',nome)
        #imprime na tela o numero de thrusters.
        print(f'Numero de thrusters do {nome}:',auv.get_numThrusters())
        #tranforma a lista de sensores em uma string separada por virgulas.
        str_sensores = ''
        for sensor in auv.get_sensores():
            str_sensores += sensor+', '
        #retira a virugla e o espaco que ficaram no final da string.
        str_sensores = str_sensores[0:len(str_sensores)-2]
        #mostra os sensores na tela.
        print(f'Os sensosres do {nome} sao:',str_sensores)
        #mostra o ano de construcao.
        print(f'O AUV {nome} foi construido em:',auv.get_ano_construcao())
        #mostra o modelo das cameras do auv.
        print(f'As cameras do {nome} sao do seguinte modelo:',auv.get_modelo_camera())

    def rankear_idade(self):
        '''Metodo para mostrar os dois modelos em um ranking de idades.'''
        #Condicional verifica qual modelo eh mias novo e imprime o resultado na tela.
        if brhue.get_ano_construcao()>lua.get_ano_construcao():
            print('AUV mais novo: BrHUE')
            print('AUV mais antigo: Lua')
        else:
            print('AUV mais novo: Lua')
            print('AUV mais antigo: BrHue')
    
    def numero_sensores(self):
        '''Metodo para imprimir a quantidade de sensores de cada modelo e a diferenca na quantidade entre os modelos.'''
        #variavel que guarda o numero de sensores do brhue, começa em zero pois sera modificada no for.
        num_sensores_brhue = 0
        #passa por todos os sensores da lista para contabiliza-los.
        for sensor in brhue.get_sensores():
            #separa a string sensor para conferir a quantidade de sensores.
            sensor = sensor.split(' ')
            try:
                #se for mais que um, a variavel qtd (quantidade) guarda essa informacao.
                qtd = int(sensor[0])
            except ValueError:
                #se a string do sensor nao comecar com um numero, considera que eh soh um daquele tipo.
                num_sensores_brhue += 1
            else:
                #se a string do sensor comecar com um numero, soma a quantidade ao numero total de sensores do brhue.
                num_sensores_brhue += qtd
        #variavel que guarda o numero de sensores da lua, começa em zero pois sera modificada no for.
        num_sensores_lua = 0
        #for igual ao for anterior.
        for sensor in lua.get_sensores():
            sensor = sensor.split(' ')
            try:
                qtd = int(sensor[0])
            except ValueError:
                num_sensores_lua += 1
            else:
                num_sensores_lua += qtd
        #mostra a quantidade de sensores de cada auv.
        print('O BrHUE tem {} sensores'.format(num_sensores_brhue))
        print('A Lua tem {} sensores'.format(num_sensores_lua))
        #condicional para verificar qual auv tem mais sensores e imprime isso na tela.
        if num_sensores_brhue > num_sensores_lua:
            print('O BrHUE possui {} sensores a mais que a Lua.'.format(num_sensores_brhue-num_sensores_lua))
        elif num_sensores_brhue == num_sensores_lua:
            print('O BrHUE e a Lua possuem o mesmo numero de sensores.')
        else:
            print('A Lua possui {} sensores a mais que o BrHUE.'.format(num_sensores_lua-num_sensores_brhue))
        
#criando objetos (que sao os auvs) da class modeloAUV.
brhue = ModeloAUV(numThrusters=6,sensores=['pressure sensor','front camera','2 ground cameras','4 hydrophones'],ano_construcao=2020,nome='BrHue',modelo_camera='Logitech C920')
lua = ModeloAUV(numThrusters=8,sensores=['external pressure sensor','internal pressure sensor','higher front camera','center front camera','lower front camera',
                'ground front camera','4 hydrophones','leak sensor'],ano_construcao=2022,nome='Lua',modelo_camera='Logitech C270')
#criando um objeto da class exibicao para chamar seus metodos.
terminal = exibicao() 