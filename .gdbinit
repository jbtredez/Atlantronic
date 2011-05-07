#set prompt \033[01;32m(gdb) \033[0m

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
	set $match_time = 0

	if systick_time_start_match > 0
		set $match_time = (systick_time-systick_time_start_match)/72000000.0f
	end

	printf "uptime %f\tmatch_time %f\n", systick_time/72000000.0f, $match_time

	echo \033[01;32m
	printf " R\t"
	if pxCurrentTCB != 0
		printf "%s\n\n", pxCurrentTCB->pcTaskName
	else
		printf "aucune\n\n"
	end
	echo \033[0m

	echo \033[01;34m
	printf "etat\tdelai\tnom\tev\tmask\tev&mask\tcpu\n"
	echo \033[0m
	set $nb_pri = sizeof pxReadyTasksLists / sizeof pxReadyTasksLists[0]
	set $i = $nb_pri - 1
	
	while $i >= 0
		set $n = pxReadyTasksLists[$i].uxNumberOfItems
		set $list_end = &pxReadyTasksLists[$i].xListEnd
		set $list_next = pxReadyTasksLists[$i].xListEnd.pxNext

		while $list_next != $list_end && $n > 0
			printf " P%i\t0\t", $i
			pTcb ((tskTCB*)$list_next->pvOwner)
			set $list_next = $list_next.pxNext
			set $n--
		end
		set $i--
	end

	set $n = xPendingReadyList.uxNumberOfItems
	set $list_end = &xPendingReadyList.xListEnd
	set $list_next = xPendingReadyList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf " PR\t0\t"
		pTcb ((tskTCB*)$list_next->pvOwner)
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xDelayedTaskList.uxNumberOfItems
	set $list_end = &xDelayedTaskList.xListEnd
	set $list_next = xDelayedTaskList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf " D\t%.2f\t", ($list_next->xItemValue - systick_time)/((double)72000)
		pTcb ((tskTCB*)$list_next->pvOwner)
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xSuspendedTaskList.uxNumberOfItems
	set $list_end = &xSuspendedTaskList.xListEnd
	set $list_next = xSuspendedTaskList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf " S\tinf\t"
		pTcb ((tskTCB*)$list_next->pvOwner)
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--
end

define pTcb
	printf "%s\t%x\t%x\t%x\t%f%%\n", $arg0->pcTaskName, $arg0->event, $arg0->eventMask, $arg0->event & $arg0->eventMask, $arg0->cpu_time_used/(double)systick_time*100
end
