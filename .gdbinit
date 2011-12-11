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
	_ptasks 0
end

define ptasks_stack
	_ptasks 1
end

define pt
	p *((tskTCB *) $arg0)
end

define ps
	p /x ((tskTCB*)  $arg0)->pxTopOfStack[0]@$arg1
	set $i=0
	while $i < $arg1
		set $addr = ((tskTCB*)  $arg0)->pxTopOfStack[$i]
		if $addr > 0x8000000 && $addr < 0x8040000
			info symbol ((tskTCB*)  $arg0)->pxTopOfStack[$i]
		end
		set $i++
	end
end

define _ptasks
	printf "Il y a %i taches\n", uxCurrentNumberOfTasks
	set $match_time = 0

	if systick_time_start_match > 0
		set $match_time = (systick_time - systick_time_start_match + systick_last_load_used - SysTick.VAL)/72000000.0f
	end

	set $static_alloc = (unsigned int)&end - (unsigned int)&__data_start__
	set $malloc_pool = (unsigned int)heap_end_for_debug - (unsigned int)&end
	set $ram_size = (unsigned int)&_ram_top - (unsigned int)&__data_start__
	printf "static alloc : %i\nmalloc pool : %i\nmain stack + non utilisée : %i\n", $static_alloc, $malloc_pool, $ram_size - $static_alloc - $malloc_pool
	printf "uptime %f (%u)   match_time %f\n", (systick_time + systick_last_load_used - SysTick.VAL)/72000000.0f, (systick_time + systick_last_load_used - SysTick.VAL), $match_time

	echo \033[01;32m
	printf "  R   "
	if pxCurrentTCB != 0
		printf "%s\n\n", pxCurrentTCB->pcTaskName
	else
		printf "aucune\n\n"
	end
	echo \033[0m

	echo \033[01;34m
	if $arg0
		printf " etat | delai (ms) |    nom   |   addr     |     ev     |    mask    |  ev & mask |  last wake |  time used |   cpu  | free stack (32 bit)  |\n"
		printf "      |            |          |            |            |            |            |            |            |    %%   | current  |   min    |\n"
	else
		printf " etat | delai (ms) |    nom   |   addr     |     ev     |    mask    |  ev & mask |  last wake |  time used |   cpu  | free stack|\n"
		printf "      |            |          |            |            |            |            |            |            |    %%   | current  |\n"
	end

	echo \033[0m
	set $nb_pri = sizeof pxReadyTasksLists / sizeof pxReadyTasksLists[0]
	set $i = $nb_pri - 1
	
	while $i >= 0
		set $n = pxReadyTasksLists[$i].uxNumberOfItems
		set $list_end = &pxReadyTasksLists[$i].xListEnd
		set $list_next = pxReadyTasksLists[$i].xListEnd.pxNext

		while $list_next != $list_end && $n > 0
			printf "  P%i  |          0 |", $i
			pTcb ((tskTCB*)$list_next->pvOwner) $arg0
			set $list_next = $list_next.pxNext
			set $n--
		end
		set $i--
	end

	set $n = xPendingReadyList.uxNumberOfItems
	set $list_end = &xPendingReadyList.xListEnd
	set $list_next = xPendingReadyList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  PR  |          0 |"
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xDelayedTaskList.uxNumberOfItems
	set $list_end = &xDelayedTaskList.xListEnd
	set $list_next = xDelayedTaskList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  D   | %10.2f |", ((double)$list_next->xItemValue - (double)systick_time - systick_last_load_used + SysTick.VAL)/((double)72000)
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xSuspendedTaskList.uxNumberOfItems
	set $list_end = &xSuspendedTaskList.xListEnd
	set $list_next = xSuspendedTaskList.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  S   |        inf |"
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

end

define pTcb
	printf " %8s | %10p | %10x | %10x | %10x | %10u | %10u | %6.3f | %8i |", $arg0->pcTaskName, $arg0, $arg0->event, $arg0->eventMask, $arg0->event & $arg0->eventMask, $arg0->last_start_tick_count, $arg0->cpu_time_used, $arg0->cpu_time_used/(double)systick_time*100, $arg0->pxTopOfStack - $arg0->pxStack

	if $arg1
		set $stack_min_free = 0
		while $arg0->pxStack[$stack_min_free] == 0xa5a5a5a5
			set $stack_min_free++
		end
		printf " %8i |\n", $stack_min_free
	else
		printf "\n"
	end
end

define plot_hokuyo
	shell rm log/log_target_hokuyo
	set logging file log/log_target_hokuyo
	set logging redirect on
	set logging on

	set $i = 0
	while $i <= 682
		printf "%f\t%f\t%f\n", hokuyo_distance[$i], hokuyo_x[$i], -hokuyo_y[$i]
		set $i++
	end

	shell ./scripts/plot_hokuyo

	set logging redirect off
	set logging off
end
