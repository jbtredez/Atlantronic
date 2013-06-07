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

	set $static_alloc = (unsigned int)&__bss_end__ - (unsigned int)&__data_start__ - sizeof(ucHeap)
	set $malloc_pool = xNextFreeByte / sizeof(ucHeap)
	set $ram_size = (unsigned int)&_ram_top - (unsigned int)&__data_start__
	printf "static alloc : %i\nmalloc pool : %i\nmain stack + non utilisée : %i\n", $static_alloc, $malloc_pool, $ram_size - $static_alloc - $malloc_pool
	printf "uptime %f (%u)   match_time %f\n", (systick_time + SysTick.LOAD - SysTick.VAL)/(1000.0f*SysTick.LOAD), (systick_time + SysTick.LOAD - SysTick.VAL), $match_time

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
		printf " etat | delai (ms) |    nom   |   addr     |  time used |   cpu  | free stack (32 bit)  |\n"
		printf "      |            |          |            |            |    %%   | current   |   min    |\n"
	else
		printf " etat | delai (ms) |    nom   |   addr     |  time used |   cpu  | free stack|\n"
		printf "      |            |          |            |            |    %%   | current   |\n"
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

	set $n = xDelayedTaskList1.uxNumberOfItems
	set $list_end = &xDelayedTaskList1.xListEnd
	set $list_next = xDelayedTaskList1.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  D1  | %10.2f |", ((double)$list_next->xItemValue / 1000)
		pTcb ((tskTCB*)$list_next->pvOwner) $arg0
		set $list_next = $list_next.pxNext
		set $n--
	end
	set $i--

	set $n = xDelayedTaskList2.uxNumberOfItems
	set $list_end = &xDelayedTaskList2.xListEnd
	set $list_next = xDelayedTaskList2.xListEnd.pxNext

	while $list_next != $list_end && $n > 0
		printf "  D2  | %10.2f |", ((double)$list_next->xItemValue / 1000)
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
	printf " %8s | %10p | %10.3f | %6.3f | %9i |", $arg0->pcTaskName, $arg0, $arg0->ulRunTimeCounter / (1000.0f * SysTick.LOAD), $arg0->ulRunTimeCounter/((double)(systick_time + SysTick.LOAD - SysTick.VAL))*100, $arg0->pxTopOfStack - $arg0->pxStack

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

# arg0 : adresse tcb
# arg1 : taille a afficher
define pstack
	set $stack = ((tskTCB*)$arg0)->pxTopOfStack
	set $i = 0
	while $i < $arg1
		if $stack[$i] > 0x8000000 && $stack[$i] < &__text_end__
			list *($stack[$i]),1
		else
			printf "%10x\n",  $stack[$i]
		end
		set $i++
	end
end
