# ESP Ethernet Component

O *ESP Ethernet* é um componente desenvolvido para modularizar e simplificar o uso das bibliotecas de rede do ESP-IDF, em particular a biblioteca **ethernet**.   

Este projeto foi desenvolvido e testado utilizando a biblioteca **ESP-IDF v4.4**.


### Ajustes e melhorias

- [ ] Permitir usuário escolha de velocidade de rede, e modo
- [ ] Possibilitar o uso de DHCP

## 💻 Pré-requisitos

É necessário que se utilize no mínimo a versão 4.4 do ESP-IDF e todas as suas dependências, seguindo o guia [Get Started - ESP IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html);

## 🚀 Incluindo o modBUS Relay

O componente é compilado juntamente com o projeto que estiver inserido, desde que esteja no caminho correto:

A forma mais indicada de se importar o componente para o projeto é utilizando o próprio git utilizando o comando:

`git submodule add`**`https://github.com/hnguima/esp_ethernet.git pasta_base_do_projeto/components/esp_ethernet`**

<span style="color:red; font-weight: bold">Modo não aconselhável ➡ baixar os arquivos manualmente</span>
<span style="color:red; font-weight: bold">AVISO:</span> Usando este modo o componente não estará mais associado ao repositório do git.

Faça o download dos arquivos deste repositório e os coloque na pasta: **`pasta_base_do_projeto/components/esp_ethernet`**

O nome da pasta criada deve sempre ser **`esp_ethernet`** e o conteúdo deste repositório deve estar dentro dela(sem nenhuma pasta anterior) 

## 🤝 Colaboradores

Agradecemos às seguintes pessoas que contribuíram para este projeto:

<table>
  <tr>
    <td align="center" style="width: 100px; vertical-align: top; line-height: 1.2; border: none!important;">
      <a href="#" style="font-size: 13px;">
        <img src="https://media-exp2.licdn.com/dms/image/C4E03AQHDCF-iuQ_krQ/profile-displayphoto-shrink_800_800/0/1588645191329?e=1662595200&v=beta&t=uWlJpnMyvzGLXq0eDbPz14uVBGWdxT6vsbbJhVnW6mc" width="100px;" alt="Henrique Guimarães" style="border-radius: 50px;"/><br>
        <b>Henrique Guimarães</b>
      </a>
    </td>
  </tr>
</table>