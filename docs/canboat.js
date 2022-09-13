function openNav()
{
  document.getElementById('sidenav').style.width        = '100px';
  document.getElementById('sidenav-closed').style.width = '0';
  document.getElementById('main').style.marginLeft      = '100px';
}

function closeNav()
{
  document.getElementById('sidenav').style.width        = '0px';
  document.getElementById('sidenav-closed').style.width = '50px';
  document.getElementById('main').style.marginLeft      = '0px';
}
