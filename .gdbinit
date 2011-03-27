define mri
	monitor reset init
end

define prog
	monitor reset init
	load
	monitor reset init
end

define ptasks
	printf "Il y a %i taches\n", uxCurrentNumberOfTasks
	if pxCurrentTCB != 0
		printf "Tache en cours : %s\n", pxCurrentTCB->pcTaskName
	else
		printf "Tache en cours : aucune\n"
	end

	set $nb_pri = sizeof pxReadyTasksLists / sizeof pxReadyTasksLists[0]
	set $i = $nb_pri - 1
	
	while $i >= 0
		printf "Priorité %i :\n", $i
		ptcblist pxReadyTasksLists[$i]
		set $i--
	end

	printf "taches en attente d'être prêtes :\n"
	ptcblist xPendingReadyList

	printf "taches en attente :\n"
	ptcblist xDelayedTaskList

	printf "taches suspendues :\n"
	ptcblist xSuspendedTaskList 
end

define ptcblist
	set $n = $arg0.uxNumberOfItems
	set $list_end = &$arg0.xListEnd
	set $list_next = $arg0.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "\t%s (event = %x)\n", ((tskTCB*) $list_next->pvOwner)->pcTaskName, ((tskTCB*) $list_next->pvOwner)->event
		set $list_next = $list_next.pxNext
		set $n--
	end
end

