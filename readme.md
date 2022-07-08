
<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

# ESP Ethernet Component
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

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
        <img src="https://media-exp2.licdn.com/dms/image/C4E03AQHDCF-iuQ_krQ/profile-displayphoto-shrink_200_200/0/1588645191329?e=1662595200&v=beta&t=wDzeBAt8J7Sxtu-2Z7e3kdyJjguH01iXHGdbl1Dl7FY" width="100px;" alt="Henrique Guimarães" style="border-radius: 50px;"/><br>
        <b>Henrique Guimarães</b>
      </a>
    </td>
  </tr>
</table>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/repo_name/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/repo_name/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/repo_name/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 