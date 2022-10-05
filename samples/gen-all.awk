BEGIN {
  FS = ",";
}
{ key = $3 "." $4 "." $5;
  done[key] = $0;
}
END {
  PROCINFO["sorted_in"]="@ind_str_asc";
  for (key in done)
  {
    print done[key];
  }
}

